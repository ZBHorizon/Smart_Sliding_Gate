/**
 * @file INA226.hpp
 * @brief Provides initialization and functionality for INA226 current sensor.
 *
 * This header declares the INA226 class which communicates with the INA226
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace SlidingGate {

    /**
     * @brief INA226 current sensor class.
     */
    class INA226 {
    public:
        /**
         * @brief Initializes the INA226 sensor.
         *
         * Sets up I2C communication and writes the calibration register.
         * @return true if initialization succeeds, false otherwise.
         */
        static bool initialize();

        /**
         * @brief Reads the current in milliamperes.

         * @return The measured current in mA.
         */
        static float read_current();

    private:
        
        inline static int calibration_value = 0;
        inline static int i2c_fd = -1;///< I2C file descriptor.
        static std::mutex i2c_mutex;  ///< Protects access via I2C.
        static int calibration_value; //!< Calibration register value.
        static float current_lsb;     //!< Conversion factor for current measurement (in Amp per LSB).
    
        /**
         * @brief Converts a 16-bit value by swapping its bytes.
         * @param val The 16-bit value.
         * @return The byte-swapped 16-bit value.
         */
        static uint16_t byteswap16(uint16_t val);

        /**
         * @brief Reads a 16-bit register from the INA226 and applies byte swapping.
         * @param reg_address The register address.
         * @return The byte-swapped 16-bit value read from the sensor.
         */
        static uint16_t read16(uint8_t reg_address);

        /**
         * @brief Writes a 16-bit value to a register in the INA226 after byte swapping.
         * @param reg_address The register address.
         * @param value The 16-bit value to write.
         */
        static void write16(uint8_t reg_address, uint16_t value);


     /**  
      * @brief INA226 calibration constants.
      */
        static constexpr float INA226_CALIBRATION_CONSTANT = 0.00512f; ///<This constant (0.00512) according to the datasheet specifications.
        static constexpr float MAX_CURRENT = 8.0f;    ///< Expected maximum current (A)
        static constexpr float R_SHUNT = 0.01f;         ///< Shunt resistor value (Ohm)
        static constexpr int I2C_ADDRESS = 0x48;        ///< INA226 I2C address
        static constexpr int CONFIG_REG = 0x00;         ///< Configuration register address
        static constexpr int SHUNT_REG = 0x01;          ///< Shunt voltage register address
        static constexpr int CURRENT_REG = 0x04;        ///< Current register address
        static constexpr int CALIB_REG = 0x05;          ///< Calibration register address

        
    };

} // namespace SlidingGate
