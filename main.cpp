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

    // Simple command loop for user interaction
    char user_input = '\0';
    bool running = true;
    while (running) {
        std::cout << "\nBefehl (o=öffnen, c=schließen, h=halb, s=stop, q=beenden): ";
        std::cin >> user_input;
        switch (user_input) {
            case 'o':
                std::cout << "Tor wird geöffnet...\n";
                Motor::open();
                break;
            case 'c':
                std::cout << "Tor wird geschlossen...\n";
                Motor::close();
                break;
            case 'h':
                std::cout << "Tor wird halb geöffnet...\n";
                Motor::half_open();
                break;
            case 's':
                std::cout << "Motor wird gestoppt...\n";
                Motor::stop();
                break;
            case 'q':
                std::cout << "Programm wird beendet...\n";
                running = false;
                break;
            default:
                std::cout << "Ungültiger Befehl.\n";
        }
        std::this_thread::sleep_for(100ms);
    }

    // Stop motor loops
    Motor::stop();
    // Detach the threads (or join if appropriate)
    motorSpeedThread.detach();
    motorPosThread.detach();

    return 0;
}
