/**
 * @file INA226.cpp
 * @brief Implements the INA226 current sensor class.
 *
 * This file implements methods for setting up the INA226 sensor, reading sensor
 * registers (with byte swapping) and converting the raw current measurement to mA.
 * The conversion uses integer arithmetic for efficiency.
 */

#include <INA226.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <mutex>

namespace SlidingGate {

    /**
     * @brief Converts a 16-bit value by swapping its bytes.
     * @param val 16-bit value.
     * @return Byte-swapped 16-bit value.
     */
    uint16_t INA226::byteswap16(uint16_t val) {
        return (val << 8) | (val >> 8);
    }

    /**
     * @brief Reads a 16-bit register from the INA226 and applies byte swapping.
     * @param reg_address The register address.
     * @return The byte-swapped 16-bit value read from the sensor.
     */
    uint16_t INA226::read16(uint8_t reg_address) {
        std::scoped_lock lock(i2c_mutex);
        uint16_t result = static_cast<uint16_t>(wiringPiI2CReadReg16(i2c_fd, reg_address));
        return byteswap16(result);
    }

    /**
     * @brief Writes a 16-bit value to a register in the INA226 after byte swapping.
     * @param reg_address The register address.
     * @param value The 16-bit value to write.
     */
    void INA226::write16(uint8_t reg_address, uint16_t value) {
        std::scoped_lock lock(i2c_mutex);
        wiringPiI2CWriteReg16(i2c_fd, reg_address, byteswap16(value));
    }

    /**
     * @brief Initializes the INA226 sensor.
     *
     * Sets up I2C communication using I2C_ADDRESS.
     * Computes the calibration value using a two-step float calculation
     * and writes it to the calibration register.
     *
     * @return true if initialization succeeds, false otherwise.
     */
    bool INA226::initialize() {
        i2c_fd = wiringPiI2CSetup(I2C_ADDRESS);
        if (i2c_fd < 0) {
            std::cerr << "INA226: Device not found on I2C bus!" << std::endl;
            return false;
        }
    
        // Calculate current_lsb based on MAX_CURRENT and 15-bit resolution.
        current_lsb = MAX_CURRENT / static_cast<float>(1 << 15);
    
        // Calculate calibration value:
        // calibration = floor(INA226_CALIBRATION_CONSTANT / (current_lsb * R_SHUNT))
        float calibration = INA226_CALIBRATION_CONSTANT / (current_lsb * R_SHUNT);
        calibration_value = static_cast<int>(std::floor(calibration));
    
        // Recalculate current_lsb using the integer calibration_value.
        current_lsb = INA226_CALIBRATION_CONSTANT / (R_SHUNT * static_cast<float>(calibration_value));
    
        // Write the calibration value to the CALIB_REG register.
        write16(CALIB_REG, static_cast<std::uint16_t>(calibration_value));
    
        return true;
    }
    /**
     * @brief Reads the current in milliamperes.
     *
     * Liest den Rohwert aus dem CURRENT_REG und berechnet den Strom in mA rein
     *
     * @return Der berechnete Strom in mA.
     */
    float INA226::read_current() {
        // Read the raw current register value as a signed 16-bit integer.
        int16_t raw_current = static_cast<int16_t>(read16(CURRENT_REG));
        // Convert to mA: raw_current (in LSB) * current_lsb (in A per LSB) * 1000
        return static_cast<float>(raw_current) * 1000.0f * current_lsb;
    }
    

} // namespace SlidingGate

