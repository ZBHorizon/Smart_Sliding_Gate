//! @file gpio_initialization.hpp
//! @brief Provides initialization for GPIO pins, including motor, lamps and sensors.

#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace SlidingGate {

    //! GPIO pin definitions.
    struct Pin {
        //! Manages the initialization of GPIO pins.

        //! @brief Initializes all GPIO pins used in the system.
        static bool initialize_gpio();


        static constexpr int PWM = 1;  //!< Hardware PWM pin.
        static constexpr int DIRECTION = 4;  //!< Direction control pin.
        static constexpr int OPEN_SWITCH = 21; //!< Left end switch.
        static constexpr int CLOSE_SWITCH = 22; //!< Right end switch.
        static constexpr int REMOTE_A = 3;  //!< Open remote button.
        static constexpr int REMOTE_B = 2;  //!< Half open remote button.
        static constexpr int REMOTE_C = 12; //!< Close remote button.
        static constexpr int REMOTE_D = 13; //!< Garden door remote button.
        static constexpr int LAMP = 15; //!< Gate lamp.
        static constexpr int GARDEN_DOOR = 16; //!< Garden door control.
        static constexpr int LIGHT_BARRIER = 5;  //!< Light barrier sensor.
    };

} // namespace SlidingGate
