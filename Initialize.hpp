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

        static constexpr int PWM = 1;          //!< Hardware PWM pin.
        static constexpr int DIRECTION = 4;      //!< Direction control pin.
        static constexpr int OPEN_SWITCH = 27;   //!< Open end switch.
        static constexpr int CLOSE_SWITCH = 28;  //!< Close end switch.
        static constexpr int REMOTE_A = 21;         //!< Open remote button.
        static constexpr int REMOTE_B = 22;         //!< Half-open remote button.
        static constexpr int REMOTE_C = 23;        //!< Close remote button.
        static constexpr int REMOTE_D = 24;        //!< Garden door remote button.
        static constexpr int LAMP = 26;            //!< Gate lamp.
        static constexpr int GARDEN_DOOR = 6;     //!< Garden door control.
        static constexpr int LIGHT_BARRIER = 29;    //!< Light barrier sensor.
    };

} // namespace SlidingGate
