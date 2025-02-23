#include <iostream>
#include <thread>
#include <chrono>
#include <Motor.hpp>
#include <Initialize.hpp>
#include <INA226.hpp>
#include <job.hpp>

using namespace std::chrono;
using namespace SlidingGate;

int main() {
    // Initialize GPIO and INA226 sensor and check for errors.
    if (!Pin::initialize_gpio()) {
        std::cerr << "GPIO initialization failed!" << std::endl;
        return 1;
    }
    if (!INA226::initialize()) {
        std::cerr << "INA226 initialization failed!" << std::endl;
        return 1;
    }

    // Start motor loop in a separate thread.
    std::thread motorThread(&Motor::motor_loop);

    while (true) {
        // Ask user to calibrate if not calibrated yet.
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
        } else {
            // Ask user at what percentage to target the gate.
            std::cout << "Enter the percentage to target the gate or 's' to stop: ";
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
        }
        std::this_thread::sleep_for(50ms);
    }

    motorThread.join();
    return 0;
}