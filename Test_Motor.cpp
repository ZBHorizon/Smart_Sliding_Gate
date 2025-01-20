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
            //! Maximum motor speed
            inline static uint8_t max_speed = 100;
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
        
        inline static uint8_t current_position = 0;
        
        inline static uint8_t desired_position = 0;

        /*!
         * \brief Calculates brake time in ms based on the given speed.
         */
        static milliseconds calculate_brake_time(int8_t speed) {

            // Number of steps from speed down to 0
            uint8_t steps = std::abs(speed)/Param::step;

            // Total brake time = steps * time-per-step
            milliseconds brake_time_ms = steps * Param::motor_ramp;
            return brake_time_ms;
        }

        


        static void open() {
            std::lock_guard<std::mutex> lock(motor_mutex);
        
        }
        static void close() {
            std::lock_guard<std::mutex> lock(motor_mutex);
            
        }
        static void stop() {
            std::lock_guard<std::mutex> lock(motor_mutex);
            current_state = STOP;
        }
        static void half_open() {
            std::lock_guard<std::mutex> lock(motor_mutex);
            
        }
        static void move_to_position(uint8_t position) {
            std::lock_guard<std::mutex> lock(motor_mutex);
            
        }
        /*!
        * \brief Continuously adjusts current_position to match desired_position (0..100%).
        *        Uses steady_clock to measure real elapsed time.
        *
        * This loop runs indefinitely. If current_position != desired_position, it "moves" toward it.
        * We assume time_to_open is the total duration from 0% to 100%.
        * For forward movement:
        *   - start_position_time = (current_position/100)*time_to_open
        *   - end_position_time   = (desired_position/100)*time_to_open
        *
        * Then we measure real elapsed time from motion_start to compute the new position.
        * We also implement a simple brake logic if close to the final target.
        */
        static void motor_position_loop() {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);

                    // If both are zero, do nothing for a while
                    if (desired_position == 0 && current_position == 0) {
                        std::this_thread::sleep_for(200ms);
                        continue;
                    }

                    // Only move if current_position != desired_position
                    if (current_position != desired_position) {

                        // Determine the total "time offset" for the desired_position
                        milliseconds desired_time = (time_to_open * desired_position) / 100;

                        // Convert current_position to an equivalent "time offset" as well
                        milliseconds current_time = (time_to_open * current_position) / 100;

                        // Calculate how long before we need to brake
                        milliseconds brake_time = calculate_brake_time(Param::max_speed);

                        // Capture the real clock time at the moment we start moving
                        auto motion_start = steady_clock::now();

                        // -----------------------
                        // Moving Forward
                        // -----------------------
                        if (current_position < desired_position) {
                            desired_speed = Param::max_speed;  // Full speed initially

                            // Continue until we reach or exceed the desired_position
                            while (true) {
                                // Calculate how much REAL time has elapsed since motion_start
                                auto now = steady_clock::now();
                                auto elapsed = duration_cast<milliseconds>(now - motion_start);

                                // current_elapsed_ms is how many ms we add to our starting offset
                                long long current_elapsed_ms = elapsed.count();

                                // Combine with our original offset
                                long long total_ms = static_cast<long long>(time_to_open.count());
                                long long cur_time_in_ms = static_cast<long long>(current_time.count()) + current_elapsed_ms;

                                // Clamp so we don't exceed total range
                                if (cur_time_in_ms > total_ms) {
                                    cur_time_in_ms = total_ms;
                                }

                                // If we're close to desired_time, apply brake
                                // e.g. if (desired_time - cur_time_in_ms) < brake_time => slow down
                                if (cur_time_in_ms < desired_time.count()) {
                                    long long distance_to_target = desired_time.count() - cur_time_in_ms;
                                    if (distance_to_target < brake_time.count()) {
                                        desired_speed = 10; // slower speed
                                    }
                                }

                                // Compute new position as percentage of time_to_open
                                long long new_position = (cur_time_in_ms * 100LL) / total_ms;
                                if (new_position > 100) new_position = 100; // clamp

                                current_position = static_cast<std::uint8_t>(new_position);

                                // Check if we have reached or exceeded the desired_position
                                if (current_position >= desired_position) {
                                    current_position = desired_position;
                                    break;
                                }

                                std::this_thread::sleep_for(1ms);
                            }
                        }
                        // -----------------------
                        // Moving Backward
                        // -----------------------
                        else {
                            desired_speed = -Param::max_speed;  // Negative speed initially

                            // Move backward until we reach or go below the desired_position
                            while (true) {
                                auto now = steady_clock::now();
                                auto elapsed = duration_cast<milliseconds>(now - motion_start);

                                long long current_elapsed_ms = elapsed.count();
                                long long total_ms = static_cast<long long>(time_to_open.count());

                                // For backward motion, we SUBTRACT from current_time based on elapsed
                                // If current_time was 1500 ms at start, after 100 ms real time => 1400 ms
                                long long cur_time_in_ms = static_cast<long long>(current_time.count()) - current_elapsed_ms;

                                // We can't go below 0 ms
                                if (cur_time_in_ms < 0) {
                                    cur_time_in_ms = 0;
                                }

                                // If we're close to the desired_time, apply a "brake"
                                // e.g. if cur_time_in_ms - desired_time < brake_time => slow down
                                long long desired_time_ms = desired_time.count();
                                if (cur_time_in_ms > desired_time_ms) {
                                    long long distance_to_target = cur_time_in_ms - desired_time_ms;
                                    if (distance_to_target < brake_time.count()) {
                                        desired_speed = -10; // slower negative speed
                                    }
                                }

                                // Convert cur_time_in_ms back to a position
                                long long new_position = (cur_time_in_ms * 100LL) / total_ms;
                                if (new_position < 0) new_position = 0; // clamp min

                                current_position = static_cast<std::uint8_t>(new_position);

                                // If we've reached or gone below desired_position, clamp
                                if (current_position <= desired_position) {
                                    current_position = desired_position;
                                    break;
                                }

                                std::this_thread::sleep_for(1ms);
                            }
                        }
                    }
                }

                // Prevent 100% CPU if there's no movement
                std::this_thread::sleep_for(1ms);
            }
        }
        /*!
 * \brief Moves from current_position to target_position using the time-based offset approach.
 *        This function will block until target_position is reached or until desired_position changes.
 */
        static void move_position_time_based(std::uint8_t start_pos, std::uint8_t target_pos) {
            using namespace std::chrono;

            if (start_pos == target_pos) return;

            // Convert positions to time offsets
            milliseconds start_time_ms = (time_to_open * start_pos) / 100;
            milliseconds end_time_ms = (time_to_open * target_pos) / 100;

            bool moving_forward = (target_pos > start_pos);

            // Set initial speed
            desired_speed = moving_forward ? Param::max_speed : -Param::max_speed;

            // Mark the motion start
            auto motion_start = steady_clock::now();

            while (true) {
                // If user changed desired_position mid-move, break
                if (desired_position != target_pos) {
                    break;
                }

                auto now = steady_clock::now();
                auto delta = duration_cast<milliseconds>(now - motion_start).count();

                long long total_ms = static_cast<long long>(time_to_open.count());
                long long cur_offset = static_cast<long long>(start_time_ms.count());

                // Depending on direction
                long long current_time_in_ms = moving_forward ? (cur_offset + delta)
                    : (cur_offset - delta);

                // Clamp
                if (current_time_in_ms < 0) current_time_in_ms = 0;
                if (current_time_in_ms > total_ms) current_time_in_ms = total_ms;

                // Possibly reduce speed if close to the end_time
                milliseconds brake_time = calculate_brake_time(Param::max_speed);

                long long target_offset_ms = end_time_ms.count();
                long long distance_to_target = moving_forward
                    ? (target_offset_ms - current_time_in_ms)
                    : (current_time_in_ms - target_offset_ms);

                if (distance_to_target < 0) distance_to_target = 0; // no negative
                if (distance_to_target < brake_time.count()) {
                    desired_speed = moving_forward ? 10 : -10;
                }

                // Compute new position
                long long new_pos = (current_time_in_ms * 100LL) / total_ms;
                if (new_pos < 0) new_pos = 0;
                if (new_pos > 100) new_pos = 100;

                current_position = static_cast<std::uint8_t>(new_pos);

                // Check if we've reached or passed target
                if (moving_forward) {
                    if (current_position >= target_pos) {
                        current_position = target_pos;
                        break;
                    }
                }
                else {
                    if (current_position <= target_pos) {
                        current_position = target_pos;
                        break;
                    }
                }

                std::this_thread::sleep_for(1ms);
            }
            // Once done, set desired_speed=0 for a clean stop 
            if (current_position == target_pos) {
                desired_speed = 0;
            }
        }

        static void motor_position_loop() {
            using namespace std::chrono;

            while (true) {
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);

                    // If both are zero, do nothing for a while
                    if (desired_position == 0 && current_position == 0) {
                        std::this_thread::sleep_for(200ms);
                        continue;
                    }

                    // If current_position already matches desired_position, nothing to do
                    if (current_position == desired_position) {
                        std::this_thread::sleep_for(50ms);
                        continue;
                    }

                    // ----------------------------------------------------------------
                    // Special handling: if user wants fully CLOSED (0%) or fully OPEN (100%)
                    // We'll do a two-step approach:
                    // 1) time-based movement close to the extreme
                    // 2) final slow approach to confirm end switch
                    // ----------------------------------------------------------------
                    if (desired_position == 0) {
                        // 1) Time-based approach to "almost 0%"
                        //    Suppose we treat e.g. 5% as "near enough" for final slow approach
                        const std::uint8_t near_threshold = 5;
                        if (current_position > near_threshold) {
                            // Perform a partial time-based move to 5%
                            std::uint8_t temp_target = near_threshold;
                            desired_speed = -Param::max_speed;

                            move_position_time_based(current_position, temp_target);
                            // move_position_time_based is a helper function shown below
                        }

                        // 2) Final slow approach to hardware switch
                        desired_speed = -10; // or some slow negative speed
                        while (!digitalRead(Pin::CLOSE_SWITCH)) {
                            // If the user changed desired_position mid-way, break out:
                            if (desired_position != 0) break;
                            std::this_thread::sleep_for(1ms);
                        }
                        // Switch is triggered => physically at 0%
                        desired_speed = 0;
                        current_position = 0;
                        continue;  // done handling 0%
                    }
                    else if (desired_position == 100) {
                        // 1) Time-based approach to "almost 100%"
                        const std::uint8_t near_threshold = 95;
                        if (current_position < near_threshold) {
                            std::uint8_t temp_target = near_threshold;
                            desired_speed = Param::max_speed;

                            move_position_time_based(current_position, temp_target);
                        }

                        // 2) Final slow approach to hardware switch
                        desired_speed = 10;
                        while (!digitalRead(Pin::OPEN_SWITCH)) {
                            if (desired_position != 100) break;
                            std::this_thread::sleep_for(1ms);
                        }
                        desired_speed = 0;
                        current_position = 100;
                        continue;  // done handling 100%
                    }

                    // ----------------------------------------------------------------
                    // Partial move (1..99%) => pure time-based approach
                    // ----------------------------------------------------------------
                    move_position_time_based(current_position, desired_position);
                }

                // Prevent 100% CPU if there's no movement
                std::this_thread::sleep_for(1ms);
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
                if (desired_speed == 0 && current_speed == 0) {
                    std::this_thread::sleep_for(200ms);
                    continue;
                };

                // check if we are at the end of the gate
                if (current_speed > 0 && digitalRead(Pin::OPEN_SWITCH) || desired_speed < 0 && digitalRead(Pin::CLOSE_SWITCH)) {
                    current_speed = 0;
                    desired_speed = 0;
                    pwmWrite(Pin::PWM, 0);
                    continue;
                };

                // check if the light barrier is triggered
                if (current_speed > 0 && digitalRead(Pin::LIGHT_BARRIER)) {
                    current_speed = 0;
                    pwmWrite(Pin::PWM, 0);
                    //slowly open the gate
                    desired_speed = -30;
                };

                // check if the motor power is too high
                if (current_speed > 0 && analogRead(Pin::MOTOR_POWER) > 1) {
                    std::this_thread::sleep_for(500ms);
                    if (analogRead(Pin::MOTOR_POWER) > 1) {
                        current_speed = 0;
                        pwmWrite(Pin::PWM, 0);
                        //slowly open the gate
                        desired_speed = -30;
                    }
                };
                
                // Step current_speed toward desired_speed
                if (current_speed != desired_speed) {
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
        /*!
        * \brief Performs motor timing calibration.
        *        Moves gate from closed to open and vice versa, measuring times.
        *
        * After measuring time_to_open and time_to_close at Param::calibration_speed,
        * we scale them to represent the times at Param::max_speed, assuming a linear
        * relationship between speed and travel time:
        *
        *   time_at_max_speed = time_measured * (calibration_speed / max_speed)
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
            time_to_open = 0ms;
            time_to_close = 0ms;
            CalibrationStep calibration_step = check_position;

            while (true) {
                switch (calibration_step) {

                case check_position: {
                    // Check which limit switch is currently active
                    if (digitalRead(Pin::OPEN_SWITCH)) {
                        calibration_step = measure_time_to_fully_open;
                    }
                    else if (digitalRead(Pin::CLOSE_SWITCH)) {
                        calibration_step = measure_time_to_fully_close;
                    }
                    else if (!digitalRead(Pin::OPEN_SWITCH) && !digitalRead(Pin::CLOSE_SWITCH)) {
                        calibration_step = move_to_starting_position;
                    }
                } break;

                case move_to_starting_position: {
                    // Move gate backwards until OPEN_SWITCH is triggered or we detect a boundary
                    desired_speed = -Param::calibration_speed;
                    if (digitalRead(Pin::OPEN_SWITCH)) {
                        calibration_step = check_position;
                    }
                } break;

                case measure_time_to_fully_open: {
                    // Move gate forward and measure how long it takes until OPEN_SWITCH is triggered
                    desired_speed = Param::calibration_speed;

                    // Mark the start time
                    auto start = steady_clock::now();

                    // Wait until the OPEN_SWITCH becomes active
                    while (!digitalRead(Pin::OPEN_SWITCH)) {
                        std::this_thread::sleep_for(1ms);
                    }

                    // Mark the end time and compute the duration
                    auto end = steady_clock::now();
                    time_to_open = duration_cast<milliseconds>(end - start);

                    // If time_to_close is already measured, we have both => scale & finish
                    if (time_to_close != 0ms) {
                        // Scale the measured times to reflect max_speed instead of calibration_speed
                        double factor = static_cast<double>(Param::calibration_speed)
                            / static_cast<double>(Param::max_speed);

                        // Temporarily store measured times
                        auto measured_open = time_to_open;
                        auto measured_close = time_to_close;

                        // Scale them
                        time_to_open = duration_cast<milliseconds>(
                            measured_open * factor);
                        time_to_close = duration_cast<milliseconds>(
                            measured_close * factor);

                        is_calibrated = true;
                        return; // Done
                    }

                    // Otherwise, move on to measure closing time
                    calibration_step = measure_time_to_fully_close;
                } break;

                case measure_time_to_fully_close: {
                    // Move gate backward and measure how long it takes until CLOSE_SWITCH is triggered
                    desired_speed = -Param::calibration_speed;

                    auto start = steady_clock::now();

                    while (!digitalRead(Pin::CLOSE_SWITCH)) {
                        std::this_thread::sleep_for(1ms);
                    }

                    auto end = std::chrono::steady_clock::now();
                    time_to_close = duration_cast<milliseconds>(end - start);

                    // If time_to_open is already measured, we have both => scale & finish
                    if (time_to_open != 0ms) {
                        double factor = static_cast<double>(Param::calibration_speed)
                            / static_cast<double>(Param::max_speed);

                        auto measured_open = time_to_open;
                        auto measured_close = time_to_close;

                        time_to_open = duration_cast<milliseconds>(
                            measured_open * factor);
                        time_to_close = duration_cast<milliseconds>(
                            measured_close * factor);

                        is_calibrated = true;
                        return; // Done
                    }
                    // Otherwise, measure opening again
                    calibration_step = measure_time_to_fully_open;
                } break;
                }
            }

            // NOTE: Because the code returns inside measure_time_to_fully_open/close,
            // we might never reach this line, but it's left here if you change the logic.
            if (time_to_open != 0ms && time_to_close != 0ms) {
                is_calibrated = true;
            }
        }

    private:

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
