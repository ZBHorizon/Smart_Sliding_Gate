#include "Initialize.h"
#include <iostream>
#include <cstdlib>


namespace ControlSystem {
    void InitializeGPIO() {
        // Initialize WiringPi
        if (wiringPiSetup() == -1) {
            std::cerr << "Failed to initialize WiringPi.\n";
            std::exit(1);
        }

        // Configure PWM
        pinMode(Pin::PWM, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(1024);
        pwmSetClock(1);

        // Configure Output Pins
        pinMode(Pin::DIRECTION, OUTPUT);
        pinMode(Pin::LAMP, OUTPUT);
        pinMode(Pin::RESERVE_OUT, OUTPUT);

        // Configure Input Pins with Pull-Up Resistors
        pinMode(Pin::LEFT_END, INPUT);
        pinMode(Pin::RIGHT_END, INPUT);
        pullUpDnControl(Pin::LEFT_END, PUD_UP);
        pullUpDnControl(Pin::RIGHT_END, PUD_UP);

        pinMode(Pin::REMOTE_A, INPUT);
        pinMode(Pin::REMOTE_B, INPUT);
        pinMode(Pin::REMOTE_C, INPUT);
        pinMode(Pin::REMOTE_D, INPUT);
        pullUpDnControl(Pin::REMOTE_A, PUD_UP);
        pullUpDnControl(Pin::REMOTE_B, PUD_UP);
        pullUpDnControl(Pin::REMOTE_C, PUD_UP);
        pullUpDnControl(Pin::REMOTE_D, PUD_UP);

        pinMode(Pin::LIGHT_BARRIER, INPUT);
        pullUpDnControl(Pin::LIGHT_BARRIER, PUD_UP);

        pinMode(Pin::RESERVE_IN, INPUT);
        pullUpDnControl(Pin::RESERVE_IN, PUD_UP);

        std::cout << "GPIO initialized successfully.\n";
    }
}
