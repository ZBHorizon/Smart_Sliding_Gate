#include <Initialize.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

//TODO überall std davor schreiben
using std::runtime_error;
//scoped_lock benutzen oder unique_lock
using std::lock_guard;

namespace SlidingGate {

    //========================================================================
    // Pin::Manager Implementation
    //========================================================================

    bool Pin::initialize_gpio() {
        if (wiringPiSetup() == -1) {
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

        pinMode(Pin::LIGHT_BARRIER, INPUT);
        pullUpDnControl(Pin::LIGHT_BARRIER, PUD_UP);

        std::cout << "GPIO initialized successfully.\n";
        return true;
    }

} // namespace SlidingGate
