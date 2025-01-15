﻿#include <iostream>
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

#include "Motor.h"
#include "Initialize.h"
#include "config_manager.h"


namespace motor_speed {
    static std::mutex speed_mutex;
    static std::int32_t current_speed = 0;  //!< Current motor speed (-1023..1023)
    static std::int32_t desired_speed = 0; //!< Desired motor speed

    /*!
        * \brief Immediately stops the motor.
        *        Sets PWM to zero and speed to zero.
        */
    void stop_motor()
    {
        std::lock_guard<std::mutex> lock(speed_mutex);
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
            std::lock_guard<std::mutex> lock(speed_mutex);

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
        std::lock_guard<std::mutex> lock(speed_mutex);
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
            std::lock_guard<std::mutex> lock(speed_mutex);
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
    void isr_left_end()
    {
        stop_motor();
    }

    /*!
        * \brief ISR callback for the right end sensor.
        */
    void isr_right_end()
    {
        stop_motor();
    }

    /*!
        * \brief ISR callback for the light barrier sensor.
        */
    void isr_light_barrier()
    {
        stop_motor();
		motor_position::goto_position(0);
    }

} 
//TODO : Implement the motor_position Control
namespace motor_position {
	static std::mutex position_mutex;
	static std::int32_t current_position = 0;  //!< Current motor position in %
	static std::int32_t desired_position = 0;  //!< Desired motor position in %
	static bool is_calibrated = false;  //!< True if the motor is calibrated
	static std::int32_t time_to_open = 0;  //!< Time it takes to fully open the gate
	static std::int32_t time_to_close = 0;  //!< Time it takes to fully close the gate
    
	// Calibrates the motor position by moving it to one of the end switches.
    void calibrate_poition() {

    }

 
	// Calibrate Timing 0% - 100% by moving the motor from the right end switch to the left end switch and measiring the time
	void calibrate_timing() {
        time_to_open = 0;
        time_to_close = 0;

        /**********

        Help 🆘🤯😵‍💫 Ideas?

        ***********/

		// run the motor Backwards until the left end switch is pressed, if one of the end switches is pressed, skip to the next part
        if (digitalRead(Pin::LEFT_END) && digitalRead(Pin::RIGHT_END)) motor_speed::set_desired_speed(-callibration_speed); 
        
		//wait till it reached the left end switch

		//if the Right end switch is pressed, skip to the next part and come back later

		//run the motor forwards if the right end switch is pressed
        motor_speed::set_desired_speed(callibration_speed);
        //measure the time until the right end switch is pressed
        while (digitalRead(Pin::LEFT_END)) {
            time_to_open++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        //save the time 
        save_variable("calibration_time", time_to_open);

		//wait till it reached the right end switch

		//run the motor backwards if the left end switch is pressed
        motor_speed::set_desired_speed(-callibration_speed);
        //measure the time until the left end switch is pressed
        while (digitalRead(Pin::RIGHT_END)) {
            time_to_close++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        //save the time
        save_variable("calibration_time", time_to_close);
        
	}
    
    //Sets the desired motor position in steps.
	void goto_position(std::int32_t position)
	{
		std::lock_guard<std::mutex> lock(position_mutex);
		desired_position = position;
	}

    // thread function that continuously adjusts the current motor position
	void motor_position_loop()
	{
		while (true) {
			{
				std::lock_guard<std::mutex> lock(position_mutex);

			}
			// Delay for a smooth transition
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

int main()
{

    // Initialize wiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize wiringPi!" << std::endl;
        return 1;
    }

    wiringPiISR(Pin::LEFT_END, INT_EDGE_RISING, motor_speed::isr_left_end);
    wiringPiISR(Pin::RIGHT_END, INT_EDGE_RISING, motor_speed::isr_right_end);
    wiringPiISR(Pin::LIGHT_BARRIER, INT_EDGE_RISING, motor_speed::isr_light_barrier);

    // Start the motor control thread
    std::thread control_thread(motor_speed::motor_speed_loop);


    control_thread.detach(); 

    return 0;
}
