//! @file gpio_initialization.hpp
//! Provides initialization for GPIO pins, including motor, lamps, sensors, etc.

#pragma once
#include <wiringPi.h>

namespace ControlSystem {
    //! Initializes all GPIO pins used in the system.
    void InitializeGPIO();
}
