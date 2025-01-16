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
#include <Motor.hpp>
#include <Initialize.hpp>
#include <Event_handler.hpp>

using namespace std::chrono_literals;

namespace SlidingGate {
    std::chrono::milliseconds Ramp::start_Motor = 5ms;
    std::chrono::milliseconds Ramp::stop_Motor = 4ms;
    int MotorPosition::calibration_speed = 100;
    
    class motor{
        static std::mutex motor_mutex;
        static std::int32_t current_speed = 0;  //!< Current motor speed (-1023..1023)
        static std::int32_t desired_speed = 0; //!< Desired motor speed
        static std::int32_t current_position = 0;  //!< Current motor position in %
        static std::int32_t desired_position = 0;  //!< Desired motor position in %
        static bool is_calibrated = false;  //!< True if the motor is calibrated
        static std::chrono::milliseconds time_to_open = 0ms;  //!< Time it takes to fully open the gate
        static std::chrono::milliseconds time_to_close = 0ms;  //!< Time it takes to fully close the gate

         /*!
         * \brief Immediately stops the motor.
         *        Sets PWM to zero and speed to zero.
         */
        void stop_motor()
        {
            std::lock_guard<std::mutex> lock(motor_mutex);
            pwmWrite(Pin::PWM, 0);
            current_speed = 0;
        }

        /*!
            * \brief Smoothly stops the motor by ramping down speed.
            *        The function uses a small delay to simulate gradual slowdown.
            */
        void soft_stop()
        {
            while (true) {
                std::lock_guard<std::mutex> lock(motor_mutex);

                if (current_speed == 0) {
                    // Already stopped
                    pwmWrite(Pin::PWM, 0);
                    digitalWrite(Pin::DIRECTION, LOW);
                    break;
                }

                // Ramp speed toward 0
                if (current_speed < 0) {
                    current_speed += 1;
                }
                else {
                    current_speed -= 1;
                }

                pwmWrite(Pin::PWM, std::abs(current_speed));

                std::this_thread::sleep_for(std::chrono::milliseconds(Ramp::stop_Motor));
            }
        }

        /*!
            * \brief Sets the desired motor speed.
            *        The actual speed is adjusted in a separate control loop/thread.
            * \param speed The new desired speed, -1023 to +1023.
            */
        void set_desired_speed(std::int32_t speed)
        {
            std::lock_guard<std::mutex> lock(motor_mutex);
            desired_speed = speed;
        }

        /*!
            * \brief A thread function that continuously adjusts the current motor speed
            *        to match the desired speed smoothly.
            *        This allows you to call set_desired_speed() asynchronously.
            */
        void motor_speed_loop()
        {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    // Check  Ends:
                    if (desired_speed > 0 && digitalRead(Pin::LEFT_END)) return;
                    if (desired_speed < 0 && (digitalRead(Pin::RIGHT_END) || digitalRead(Pin::LIGHT_BARRIER))) return;

                    if (desired_speed == 0) {
                        soft_stop();
                        return;
                    }

                    // Stepp into desired speed
                    if (current_speed < desired_speed) current_speed++;
                    else if (current_speed > desired_speed) current_speed--;

                    // set direction at 0 speed
                    if (current_speed == 0) digitalWrite(Pin::DIRECTION, (desired_speed >= 0) ? LOW : HIGH);

                    pwmWrite(Pin::PWM, std::abs(current_speed));

                }

                // Delay for a smooth transition (replace Ramp::start_Motor
                // with a real value in milliseconds)
                std::this_thread::sleep_for(std::chrono::milliseconds(Ramp::start_Motor));
            }
        }

        /*!
            * \brief ISR callback for the left end sensor.
            */
        volatile bool left_end_triggered = false;
        void isr_left_end()
        {
            left_end_triggered = true;
            stop_motor();
        }

        /*!
            * \brief ISR callback for the right end sensor.
            */
        volatile bool right_end_triggered = false;
        void isr_right_end()
        {
            left_end_triggered = true;
            stop_motor();
        }

        /*!
            * \brief ISR callback for the light barrier sensor.
            */
        volatile bool right_end_triggered = false;
        void isr_light_barrier()
        {
            stop_motor();
            goto_position(0);
        }

        // Calibrate Timing 0% - 100% by moving the motor from the right end switch to the left end switch and measiring the time
        void calibrate_timing() {
            std::lock_guard<std::mutex> lock(motor_mutex);
            enum CalibrationStep {
                move_to_starting_position,
                check_position,
                measure_time_to_fully_open,
                measure_time_to_fully_close,
            };

            time_to_open = 0ms;
            time_to_close = 0ms;
            CalibrationStep calibration_step = check_position;

            while (true) {
                switch (calibration_step) {
                    case check_position: {
                        if (digitalRead(Pin::LEFT_END))  calibration_step = measure_time_to_fully_open;
                        if (digitalRead(Pin::RIGHT_END)) calibration_step = measure_time_to_fully_close;
                        if (!digitalRead(Pin::LEFT_END) && !digitalRead(Pin::RIGHT_END)) calibration_step = move_to_starting_position;
                    } break;
                    case move_to_starting_position: {
                        // run the motor Backwards until the left end switch is pressed, if one of the end switches is pressed, skip to the next part
                        set_desired_speed(-MotorPosition::calibration_speed);
                        if (digitalRead(Pin::LEFT_END)) calibration_step = check_position;
                    } break;
                    case measure_time_to_fully_open: {
                        left_end_triggered = false;
                        //run the motor forwards if the left end switch is pressed
                        set_desired_speed(MotorPosition::calibration_speed);
                        //measure the time until the left end switch is pressed
                        while (!left_end_triggered) {
                            time_to_open++;
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        //save the time 
                        
                        //start calibration time to close if not calibrated yet
                        if (time_to_close != 0ms)  return; 
                        calibration_step = measure_time_to_fully_close;
                    } break;
                    case measure_time_to_fully_close: {
                        right_end_triggered = false;
                        //run the motor backwards if the right end switch is pressed
                        set_desired_speed(-MotorPosition::calibration_speed);
                        //measure the time until the right end switch is pressed
                        while (!left_end_triggered) {
                            time_to_close++;
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        //save the time
                        
                        //start calibration time to close if not calibrated yet
                        if (time_to_open != 0ms) return;
                        calibration_step = measure_time_to_fully_open;
                    } break;
                }
            }
        }
    
        //Sets the desired motor position in %.
        void goto_position(std::int32_t position)
        {
            std::lock_guard<std::mutex> lock(motor_mutex);
        
        }

        // thread function that continuously adjusts the current motor position
        void motor_position_loop()
        {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    // 
                }
                    // Delay for a smooth transition
                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    };
}

int main()
{
	using namespace SlidingGate;
    // Initialize wiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize wiringPi!" << std::endl;
        return 1;
    }

    wiringPiISR(Pin::LEFT_END, INT_EDGE_RISING, motor::isr_left_end);
    wiringPiISR(Pin::RIGHT_END, INT_EDGE_RISING, motor::isr_right_end);
    wiringPiISR(Pin::LIGHT_BARRIER, INT_EDGE_RISING, motor::isr_light_barrier);

    // Start the motor control thread
    std::thread control_thread1(motor::motor_speed_loop);
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    //control_thread1.detach(); 

    //return 0;
}
