//! @file Initialize.hpp
//! @brief Provides GPIO initialization for motor control, lamps, and sensors.

#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace SlidingGate {

    //! GPIO pin definitions.
    struct Pin {
        //! @brief Initializes all GPIO pins used in the system.
        static bool initialize_gpio();
        /*
        gpio readall
         +-----+-----+---------+------+---+---Pi 5---+---+------+---------+-----+-----+
         | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
         +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
         |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
         |   2 |   8 |   SDA.1 | ALT3 | 1 |  3 || 4  |   |      | 5v      |     |     |
         |   3 |   9 |   SCL.1 | ALT3 | 0 |  5 || 6  |   |      | 0v      |     |     |
         |   4 |   7 | GPIO. 7 |   -  | 0 |  7 || 8  | 0 |  -   | TxD     | 15  | 14  |
         |     |     |      0v |      |   |  9 || 10 | 0 |  -   | RxD     | 16  | 15  |
         |  17 |   0 | GPIO. 0 |   IN | 1 | 11 || 12 | 0 | ALT3 | GPIO. 1 | 1   | 18  |
         |  27 |   2 | GPIO. 2 |   IN | 1 | 13 || 14 |   |      | 0v      |     |     |
         |  22 |   3 | GPIO. 3 |   IN | 1 | 15 || 16 | 0 | OUT  | GPIO. 4 | 4   | 23  |
         |     |     |    3.3v |      |   | 17 || 18 | 0 |  -   | GPIO. 5 | 5   | 24  |
         |  10 |  12 |    MOSI |   -  | 0 | 19 || 20 |   |      | 0v      |     |     |
         |   9 |  13 |    MISO |   -  | 0 | 21 || 22 | 0 | OUT  | GPIO. 6 | 6   | 25  |
         |  11 |  14 |    SCLK |   -  | 0 | 23 || 24 | 0 |  -   | CE0     | 10  | 8   |
         |     |     |      0v |      |   | 25 || 26 | 0 |  -   | CE1     | 11  | 7   |
         |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
         |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
         |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | OUT  | GPIO.26 | 26  | 12  |
         |  13 |  23 | GPIO.23 |   IN | 1 | 33 || 34 |   |      | 0v      |     |     |
         |  19 |  24 | GPIO.24 |   IN | 1 | 35 || 36 | 1 | IN   | GPIO.27 | 27  | 16  |
         |  26 |  25 | GPIO.25 |   -  | 0 | 37 || 38 | 1 | IN   | GPIO.28 | 28  | 20  |
         |     |     |      0v |      |   | 39 || 40 | 1 | IN   | GPIO.29 | 29  | 21  |
         +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
         | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
         +-----+-----+---------+------+---+---Pi 5---+---+------+---------+-----+-----+
        
        */
        static constexpr int PWM = 1;          //!< Hardware PWM pin.
        static constexpr int DIRECTION = 4;      //!< Direction control pin.
        static constexpr int OPEN_SWITCH = 2;   //!< Open end switch.
        static constexpr int CLOSE_SWITCH = 0;  //!< Close end switch.
        static constexpr int LIGHT_BARRIER = 3;    //!< Light barrier sensor.
        static constexpr int REMOTE_A = 21;         //!< Open remote button.
        static constexpr int REMOTE_B = 22;         //!< Half-open remote button.
        static constexpr int REMOTE_C = 23;        //!< Close remote button.
        static constexpr int REMOTE_D = 24;        //!< Garden door remote button.
        static constexpr int LAMP = 26;            //!< Gate lamp.
        static constexpr int GARDEN_DOOR = 6;     //!< Garden door control.
        
    };

} // namespace SlidingGate
