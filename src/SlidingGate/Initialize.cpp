#include <SlidingGate/Initialize.hpp>
#include <SlidingGate/Motor.hpp>
#include <SlidingGate/Control.hpp>
#include <SlidingGate/IO.hpp>

#include <iostream>
#include <cstdlib>
#include <cmath>

namespace SlidingGate {

    //========================================================================
    // Pin Initialization Implementation
    //========================================================================

    bool Pin::initialize_gpio() {
        if (IO::wiringPiSetup() == -1) {
            std::cerr << "wiringPiSetup failed." << std::endl;
            return false;
        }

        // Setup PWM for motor control.
        IO::pinMode(Pin::PWM, PWM_OUTPUT);
        IO::pwmSetMode(PWM_MODE_MS);
        IO::pwmSetRange(128); // 128 -> 4-bit resolution; up to 4096 for 12-bit resolution.
        IO::pwmSetClock(8);   // PWM frequency: 19,200,000 / (8 * 128) ? 18,750 Hz.
        // Note: ~20 kHz is recommended for the Cytron MD20A Motor Driver.

        // Setup output pins.
        IO::pinMode(Pin::DIRECTION, OUTPUT);
        IO::pinMode(Pin::LAMP, OUTPUT);
        IO::pinMode(Pin::GARDEN_DOOR, OUTPUT);

        // Setup input pins with pull-up resistors.
        IO::pinMode(Pin::OPEN_SWITCH, INPUT);
        IO::pinMode(Pin::CLOSE_SWITCH, INPUT);
        IO::pullUpDnControl(Pin::OPEN_SWITCH, PUD_UP);
        IO::pullUpDnControl(Pin::CLOSE_SWITCH, PUD_UP);

        IO::pinMode(Pin::REMOTE_A, INPUT);
        IO::pinMode(Pin::REMOTE_C, INPUT);
        IO::pinMode(Pin::REMOTE_B, INPUT);
        IO::pinMode(Pin::REMOTE_D, INPUT);
        IO::pullUpDnControl(Pin::REMOTE_A, PUD_UP);
        IO::pullUpDnControl(Pin::REMOTE_B, PUD_UP);
        IO::pullUpDnControl(Pin::REMOTE_C, PUD_UP);
        IO::pullUpDnControl(Pin::REMOTE_D, PUD_UP);
        IO::wiringPiISR(Pin::REMOTE_A, INT_EDGE_RISING, Control::remote_a_isr);
        IO::wiringPiISR(Pin::REMOTE_B, INT_EDGE_RISING, Control::remote_b_isr);
        // IO::wiringPiISR(Pin::REMOTE_C, INT_EDGE_RISING, Control::remote_c_isr);
        IO::wiringPiISR(Pin::REMOTE_D, INT_EDGE_RISING, Control::remote_d_isr);

        IO::pinMode(Pin::LIGHT_BARRIER, INPUT);
        IO::pullUpDnControl(Pin::LIGHT_BARRIER, PUD_UP);
        IO::wiringPiISR(Pin::LIGHT_BARRIER, INT_EDGE_FALLING, Motor::light_barrier_isr);
        IO::wiringPiISR(Pin::CLOSE_SWITCH, INT_EDGE_FALLING, Motor::close_switch_isr);
        IO::wiringPiISR(Pin::OPEN_SWITCH, INT_EDGE_FALLING, Motor::open_switch_isr);
        std::cout << "GPIO initialized successfully." << std::endl;
        return true;
    }

} // namespace SlidingGate
