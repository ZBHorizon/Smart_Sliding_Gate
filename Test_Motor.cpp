#include <iostream>
#include <fstream>
#include <string>
#include <cstdint>
#include <cstdlib>    // for std::abs
#include <mutex>
#include <thread>
#include <chrono>
#include <stdexcept>

// WiringPi includes (Raspberry Pi)
#include <wiringPi.h>
#include <Initialize.hpp>

using namespace std::chrono;

namespace SlidingGate {

    //! Mutex to protect all static motor data.
    static std::mutex motor_mutex;

    /*!
     * \brief A fully static Motor class (no instances) that controls the gate.
     */
    class Motor {
    public:
        /*!
         * \brief Param timings for starting/stopping motor speed transitions.
         *        Kept static so we can reference them without an instance.
         */
        struct Param {
            //! Time delay per speed step
            inline static milliseconds motor_ramp = 1ms;
            //! Speed used in calibration (positive = forward, negative = backward)
            inline static uint8_t calibration_speed = 10;
            //! Speed threshold for range of direction change
            inline static uint8_t direction_threshold = 2;
            //! step size
            inline static uint8_t step = 1;
            //! tolerance for speed
            inline static uint8_t tolerance = 3;
        };
        

        //! Desired motor speed, set asynchronously
        inline static int8_t desired_speed = 0;
        //! Current motor speed, gradually changes to match desired
        inline static int8_t current_speed = 0;
        //! Time it takes to fully open the gate
        inline static milliseconds time_to_open = 0ms;
        //! Time it takes to fully close the gate
        inline static milliseconds time_to_close = 0ms;
        //! Flag indicating if the motor has been calibrated
        inline static bool is_calibrated = false;
        
        /*!
         * \brief Calculates brake time in ms based on the given speed.
         */
        static milliseconds calculate_brake_time(int8_t speed) {

            // Number of steps from speed down to 0
            uint8_t steps = std::abs(speed);

            // Total brake time = steps * time-per-step
            milliseconds brake_time_ms = steps * Param::motor_ramp;
            return brake_time_ms;
        }

        /*!
         * \brief Sets desired speed, waits a given duration, then sets speed back to 0.
         */
        static void run_and_stop_after_time(int8_t speed, milliseconds duration) {
            {
                std::lock_guard<std::mutex> lock(motor_mutex);
                desired_speed = speed;
            }
            std::this_thread::sleep_for(duration);
            {
                std::lock_guard<std::mutex> lock(motor_mutex);
                desired_speed = 0;
                std::cout << "Ende\n";
            }
        }

        /*!
         * \brief Sets the desired motor speed (thread-safe).
         */
        static void set_desired_speed(int8_t speed) {
            std::lock_guard<std::mutex> lock(motor_mutex);
            desired_speed = speed;
        }

        static bool check_ends() {
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (desired_speed > 0 && digitalRead(Pin::OPEN_SWITCH)|| desired_speed < 0 && digitalRead(Pin::CLOSE_SWITCH)) {
                return true;
            }
            else
            {
                return false;
            }
        }

        /*!
         * \brief Continuously matches current_speed to desired_speed.
         *        This function is meant to run in a dedicated thread.
         */
        static void motor_speed_loop() {
            while (true) {
                std::lock_guard<std::mutex> lock(motor_mutex);

                // If we are not moving, wait a bit
                if (abs(desired_speed) + abs(current_speed) == 0) {
                    std::this_thread::sleep_for(200ms);
                    continue;
                }
                // Check for errors
                if (check_ends()) {
                    current_speed = 0;
                    desired_speed = 0;
                    pwmWrite(Pin::PWM, abs(current_speed));
                    std::this_thread::sleep_for(200ms);
                    continue;
                };
                
                // Step current_speed toward desired_speed
                while (current_speed != desired_speed) {
                    // Check for errors
                    if (check_ends()) {
                        current_speed = 0;
                        desired_speed = 0;
                        pwmWrite(Pin::PWM, abs(current_speed));
                        break;
                    };
                    // If we are close enough to the desired speed, just set it
                    if (abs(current_speed - desired_speed) > Param::tolerance) {

                        // Update direction if we cross zero
                        if (abs(current_speed) < Param::direction_threshold) {
                            digitalWrite(Pin::DIRECTION, (desired_speed >= 0) ? LOW : HIGH);
                        }
                        // Update speed
                        if (current_speed < desired_speed) {
                            current_speed = current_speed + Param::step;
                        }
                        else if (current_speed > desired_speed) {
                            current_speed = current_speed - Param::step;
                        }
                    }
                    else {
                        current_speed = desired_speed;
                    }

                    // Apply PWM: absolute value in case speed is negative
                    pwmWrite(Pin::PWM, abs(current_speed));

                    // Delay for smooth Paraming
                    std::this_thread::sleep_for(Param::motor_ramp);
                }
            }
        }

        /*!
         * \brief Performs motor timing calibration.
         *        Moves gate from closed to open and vice versa, measuring times.
         */
        static void calibrate_timing() {
            std::lock_guard<std::mutex> lock(motor_mutex);

            enum CalibrationStep {
                move_to_starting_position,
                check_position,
                measure_time_to_fully_open,
                measure_time_to_fully_close,
            };

            is_calibrated = false;
            time_to_open  = 0ms;
            time_to_close = 0ms;
            CalibrationStep calibration_step = check_position;

            while (true) {
                switch (calibration_step) {
                case check_position: {
                    if (digitalRead(Pin::OPEN_SWITCH))
                        calibration_step = measure_time_to_fully_open;
                    else if (digitalRead(Pin::CLOSE_SWITCH))
                        calibration_step = measure_time_to_fully_close;
                    else if (!digitalRead(Pin::OPEN_SWITCH) && !digitalRead(Pin::CLOSE_SWITCH))
                        calibration_step = move_to_starting_position;
                } break;

                case move_to_starting_position: {
                    // Move gate backwards until OPEN_SWITCH is triggered or we detect a boundary
                    desired_speed = -Param::calibration_speed;
                    if (digitalRead(Pin::OPEN_SWITCH))
                        calibration_step = check_position;
                } break;

                case measure_time_to_fully_open: {
                    // Move gate forward
                    desired_speed = Param::calibration_speed;
                    // measure time until OPEN_SWITCH is triggered
                    while (!digitalRead(Pin::OPEN_SWITCH)) {
                        time_to_open += 1ms;
                        std::this_thread::sleep_for(1ms);
                    }
                    // If time_to_close is nonzero, we must be done
                    if (time_to_close != 0ms) {
                        // Done measuring
                        return;
                    }
                    // Otherwise continue
                    calibration_step = measure_time_to_fully_close;
                } break;

                case measure_time_to_fully_close: {
                    // Move gate backward
                    desired_speed = -Param::calibration_speed;
                    // measure time until CLOSE_SWITCH is triggered
                    while (!digitalRead(Pin::CLOSE_SWITCH)) {
                        time_to_close += 1ms;
                        std::this_thread::sleep_for(1ms);
                    }
                    // If time_to_open is nonzero, we must be done
                    if (time_to_open != 0ms) {
                        // Done measuring
                        return;
                    }
                    // Otherwise continue
                    calibration_step = measure_time_to_fully_open;
                } break;
                }
            }
             if (time_to_open != 0ms && time_to_close != 0ms) is_calibrated = true;
        }

    private:
        /*!
         * \brief Immediately stops the motor (sets PWM to zero and speed = 0).
         */
        static void motor_ramp() {
            std::lock_guard<std::mutex> lock(motor_mutex);
            pwmWrite(Pin::PWM, 0);
            current_speed = 0;
            desired_speed = 0;
        }
    };
    
    
} // namespace SlidingGate
PI_THREAD(test) {
    SlidingGate::Motor::run_and_stop_after_time(-300, 5000ms);
}
// --------------------------------------------------------------
// Main Program
// --------------------------------------------------------------
int main()
{
    using namespace SlidingGate;

    // Initialize WiringPi only once
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize wiringPi!" << std::endl;
        return 1;
    }

    // Set up GPIO pins
    Pin::Manager::InitializeGPIO();

    //! Start motor speed loop in its own thread
    std::thread control_thread(&Motor::motor_speed_loop);

    // User input
    char user_input = '\0';
    /*
    // Check calibration
    if (!Motor::is_calibrated) {
        std::cout << "Der Motor ist nicht kalibriert. Möchten Sie die Kalibrierung jetzt starten? (j/n): ";
        std::cin >> user_input;
        if (user_input == 'j' || user_input == 'J') {
            Motor::calibrate_timing();

            // Check if calibration succeeded
            if (Motor::time_to_open != 0ms && Motor::time_to_close != 0ms) {
                std::cout << "Kalibrierung abgeschlossen. Möchten Sie die Werte behalten? (j/n): ";
                std::cin >> user_input;
                if (user_input == 'j' || user_input == 'J') {
                    Motor::is_calibrated = true;
                }
                else {
                    Motor::time_to_open  = 0ms;
                    Motor::time_to_close = 0ms;
                }
            }
            else {
                std::cout << "Kalibrierung fehlgeschlagen.\n";
            }
        }
    }*/
    /*
    // Main loop for user interaction
    while (true)
    {
        std::cout << "\nWas möchten Sie tun? (o=öffnen, c=schließen, h=halb öffnen, s=stoppen, q=beenden): ";
        std::cin >> user_input;

        if (user_input == 'o') {
            // Gate open
            std::cout << "Tor öffnet...\n";
            int16_t speed = 100;

            // Calculate time till we start braking
            // (time_to_open) minus brake_time
            auto time_till_stop_ms = Motor::time_to_open 
                - milliseconds(Motor::calculate_brake_time(speed));

            if (time_till_stop_ms < 0ms) {
                time_till_stop_ms = 500ms; // fallback
            }

            Motor::run_and_stop_after_time(speed, time_till_stop_ms);
        }
        else if (user_input == 'c') {
            // Gate close
            std::cout << "Tor schließt...\n";
            int16_t speed = 100;

            auto time_till_stop_ms = Motor::time_to_open
                - milliseconds(Motor::calculate_brake_time(speed));

            if (time_till_stop_ms < 0ms) {
                time_till_stop_ms = 500ms;
            }
            // negative speed = backwards
            Motor::run_and_stop_after_time(-speed, time_till_stop_ms);
        }
        else if (user_input == 'h') {
            // Half open
            std::cout << "Tor öffnet zur Hälfte...\n";
            // Example: just run for 5000ms at speed=100
            piThreadCreate(test);
        }
        else if (user_input == 's') {
            // Stop the motor
            std::cout << "Motor wird gestoppt.\n";
            Motor::set_desired_speed(0);
        }
        else if (user_input == 'q') {
            // Quit
            std::cout << "Programm wird beendet.\n";
            break;
        }
        else {
            std::cout << "Ungültige Eingabe.\n";
        }

        // Optional pause
        std::this_thread::sleep_for(100ms);
    }
    */

    // Stop motor control thread
    Motor::set_desired_speed(0);
    if (control_thread.joinable()) {
        control_thread.join();
    }

    return 0;
}
