#include <SlidingGate/Initialize.hpp>
#include <SlidingGate/Motor.hpp>
#include <SlidingGate/Control.hpp>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include <cstdlib>
#include <cmath>

namespace SlidingGate {

    //========================================================================
    // Pin Initialization Implementation
    //========================================================================

    bool Pin::initialize_gpio() {
        if (wiringPiSetup() == -1) {
			std::cerr << "wiringPiSetup Failed." << std::endl;
            return false;
        }

        // Setup PWM for motor control.
        pinMode(Pin::PWM, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(128); // 128 -> 4-bit resolution; up to 4096 for 12-bit resolution.
        pwmSetClock(8);   // PWM frequency: 19,200,000 / (8 * 128) ≈ 18,750 Hz.
        // Note: ~20 kHz is recommended for the Cytron MD20A Motor Driver.

        // Setup output pins.
        pinMode(Pin::DIRECTION, OUTPUT);
        pinMode(Pin::LAMP, OUTPUT);
        pinMode(Pin::GARDEN_DOOR, OUTPUT);

        // Setup input pins with pull-up resistors.
        pinMode(Pin::OPEN_SWITCH, INPUT);
        pinMode(Pin::CLOSE_SWITCH, INPUT);
        pullUpDnControl(Pin::OPEN_SWITCH, PUD_UP);
        pullUpDnControl(Pin::CLOSE_SWITCH, PUD_UP);

        pinMode(Pin::REMOTE_A, INPUT);
        pinMode(Pin::REMOTE_B, INPUT);
        pinMode(Pin::REMOTE_C, INPUT);
        pinMode(Pin::REMOTE_D, INPUT);
        pullUpDnControl(Pin::REMOTE_A, PUD_UP);
        pullUpDnControl(Pin::REMOTE_B, PUD_UP);
        pullUpDnControl(Pin::REMOTE_C, PUD_UP);
        pullUpDnControl(Pin::REMOTE_D, PUD_UP);
        wiringPiISR(Pin::REMOTE_A, INT_EDGE_RISING, Control::remote_a_isr);
        wiringPiISR(Pin::REMOTE_B, INT_EDGE_RISING, Control::remote_b_isr);
        //wiringPiISR(Pin::REMOTE_C, INT_EDGE_RISING, Control::remote_c_isr);
        wiringPiISR(Pin::REMOTE_D, INT_EDGE_RISING, Control::remote_d_isr);

        pinMode(Pin::LIGHT_BARRIER, INPUT);
        pullUpDnControl(Pin::LIGHT_BARRIER, PUD_UP);
        wiringPiISR(Pin::LIGHT_BARRIER, INT_EDGE_FALLING,
                    Motor::light_barrier_isr);
        wiringPiISR(Pin::CLOSE_SWITCH, INT_EDGE_FALLING,
                    Motor::close_switch_isr);
        wiringPiISR(Pin::OPEN_SWITCH, INT_EDGE_FALLING, Motor::open_switch_isr);
        std::cout << "GPIO initialized successfully.\n";
        return true;
    }

} // namespace SlidingGate
