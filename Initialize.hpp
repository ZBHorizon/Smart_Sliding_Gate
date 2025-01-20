//! @file gpio_initialization.hpp
//! Provides initialization for GPIO pins, including motor, lamps, sensors, etc.

#pragma once
#include <wiringPi.h>

namespace SlidingGate {
    struct Pin {
        class Manager {
        public:
            //! Initializes all GPIO pins used in the system.
            static void InitializeGPIO();
        };
        /*
        +----------+------+-----+----------+-----+------+----------+
        |Function  |Mode  |Pin  |Physical  |Pin  |Mode  |Function  |
        +----------+------+-----+----------+-----+------+----------+
        |3.3V      |      |     | 1|2      |     |      |5V        |
        |          |      |8    | 3|4      |     |      |5V        |
        |          |      |9    | 5|6      |     |      |0V        |
        |          |      |7    | 7|8      |15   |      |LAMP      |
        |0V        |      |     | 9|10     |16   |      |RESERVE_OUT|
        |          |      |0    | 11|12    |1    |      |PWM        |
        |REMOTE_B  |      |2    | 13|14    |     |      |0V         |
        |REMOTE_A  |      |3    | 15|16    |4    |      |DIRECTION  |
        |3.3V      |      |     | 17|18    |5    |      |LIGHT_BARRIER|
        |REMOTE_C  |      |12   | 19|20    |     |      |0V         |
        |REMOTE_D  |      |13   | 21|22    |6    |      |RESERVE_IN  |
        |          |      |14   | 23|24    |10   |      |            |
        |0V        |      |     | 25|26    |11   |      |            |
        |          |      |30   | 27|28    |31   |      |            |
        |OPEN_SWITCH  |      |21   | 29|30    |     |      |0V          |
        |CLOSE_SWITCH |      |22   | 31|32    |26   |      |            |
        |          |      |23   | 33|34    |     |      |0V          |
        |MOTOR_POWER|      |24   | 35|36    |27   |      |            |
        |          |      |25   | 37|38    |28   |      |            |
        |0V        |      |     | 39|40    |29   |      |            |
        +----------+------+-----+----------+-----+------+----------+
        */
        static constexpr int PWM = 1;   // Hardware PWM
        static constexpr int DIRECTION = 4;   // Direction Control
        static constexpr int OPEN_SWITCH = 21;  // Left End Switch
        static constexpr int CLOSE_SWITCH = 22;  // Right End Switch
        static constexpr int REMOTE_A = 3;   // Open Remote Button
        static constexpr int REMOTE_B = 2;   // Half Open Remote Button
        static constexpr int REMOTE_C = 12;   // Close Remote Button
        static constexpr int REMOTE_D = 13;   // Garden Door Remote Button
        static constexpr int LAMP = 15;   // Gate Lamp
        static constexpr int RESERVE_OUT = 16;
        static constexpr int LIGHT_BARRIER = 5;   // Light Barrier Sensor
        static constexpr int RESERVE_IN = 6;
		static constexpr int MOTOR_POWER = 24;  // Motor Power Amperemeter Sensor
    };
}