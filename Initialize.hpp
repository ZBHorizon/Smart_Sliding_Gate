//! @file gpio_initialization.hpp
//! @brief Provides initialization for GPIO pins, including motor, lamps, sensors,
//!        and the INA226 current sensor functionality.

#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>

// Note: It is assumed that types like int16_t and uint16_t are available in the global namespace.

namespace SlidingGate {

    //! INA226 current sensor class.
    class INA226 {
    public:

        //! @brief Initializes the INA226 sensor.
        //! @throws runtime_error if the device is not found on the I2C bus.
        static void initialize();

        //! @brief Reads the current in milliamperes.
        //! @return Current measurement in mA.
        static int16_t read_current();

    private:
        
        //! @brief Reads a 16-bit register from the INA226 and converts endianness.
        //! @param reg_address Register address.
        //! @return The 16-bit value read from the sensor.
        static uint16_t read16(uint8_t reg_address);

        //! @brief Writes a 16-bit value to a register in the INA226 with proper endianness conversion.
        //! @param reg_address Register address.
        //! @param value 16-bit value to write.
        static void write16(uint8_t reg_address, uint16_t value);

        //! INA226 configuration constants.
        static constexpr float MAX_CURRENT = 8.0f; //!< Maximum current (A) expected for the motor.
        static constexpr float R_SHUNT = 0.01f;  //!< Shunt resistor value (Ohm), adjust as needed.
        static constexpr int   I2C_ADDRESS = 0x48;   //!< INA226 I2C address.
    };

    //! GPIO pin definitions.
    struct Pin {
        //! Manages the initialization of GPIO pins.
        class Manager {
        public:
            //! @brief Initializes all GPIO pins used in the system.
            //! @throws runtime_error if wiringPi initialization fails.
            static void initialize_gpio();
        };

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
