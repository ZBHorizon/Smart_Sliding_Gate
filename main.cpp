#include <iostream>
#include <thread>
#include <chrono>
#include <Motor.hpp>
#include <Initialize.hpp>
#include <INA226.hpp>
#include <job.hpp>
#include <wiringPi.h>
using namespace std::chrono;
using namespace SlidingGate;

int main() {
    // Initialize GPIO and INA226 sensor; check for errors.
    if (!Pin::initialize_gpio()) {
        std::cerr << "GPIO initialization failed!" << std::endl;
        return 1;
    }
    /*
    if (!INA226::initialize()) {
        std::cerr << "INA226 initialization failed!" << std::endl;
        return 1;
    }
    */
    // Start the motor loop in a separate thread.
    std::thread motor_thread(&Motor::motor_loop);

    while (true) {

		//std::cout << "Current: " << INA226::readCurrent_mA() << " mA" << std::endl;
        /*
        // Prompt user for calibration if not yet calibrated.
        if (!Motor::is_calibrated()) {
            std::cout << "Calibrate the motor? (y/n): ";
            char input;
            std::cin >> input;
            if (input == 'y') {
                Motor::calibrate_timing();
            } else {
                std::cout << "Motor not calibrated. Exiting program." << std::endl;
                return 0;
            }
        } else {*/
            // Ask user for the target gate percentage.
            std::cout << "Enter target gate percentage or 's' to stop: ";
            std::string input;
            std::cin >> input;
            if (input == "s") {
                job::stop_motor();
            } else {
                float target_position = std::stof(input);
                job::keyframe target {
                    .speed = 0.0f,
                    .position = target_position
                };
                job::create_job(target);
            }
            /*
            
        }
         */
        std::this_thread::sleep_for(500ms);
    }

    motor_thread.join();
    return 0;
}