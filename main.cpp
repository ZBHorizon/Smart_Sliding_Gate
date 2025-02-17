#include <iostream>
#include <thread>
#include <chrono>
#include "Motor.hpp"
#include "Initialize.hpp"

using namespace std::chrono;
using namespace SlidingGate;

int main() {
    try {
        // Initialize GPIO and INA226 sensor
        Pin::Manager::initialize_gpio();
        INA226::initialize();
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    // Start motor loops in separate threads using fully qualified names:
    std::thread motorSpeedThread(&SlidingGate::Motor::motor_speed_loop);
    std::thread motorPosThread(&SlidingGate::Motor::motor_position_loop);


    return 0;
}
