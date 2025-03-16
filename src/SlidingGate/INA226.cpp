/**
 * @file INA226.cpp
 * @brief Implements the INA226 current sensor class in English.
 *
 * This implementation initializes the INA226 sensor with a fixed configuration
 * and calibration. It sets the device to continuous mode so that current measurements
 * are updated at each conversion cycle. The readCurrent_mA() function reads the current
 * register, swaps the byte order (because the INA226 uses big-endian), and converts
 * the raw value into milliamps (mA) using a scaling factor of 1 mA/bit (or whatever
 * _CURRENT_LSB you specify).
 */

#include <SlidingGate/INA226.hpp>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <stdexcept>
#include <cstdint>
#include <limits> // for signaling_NaN()
#include <iostream>
 
namespace SlidingGate {

/**
 * @brief Swap the byte order of a 16-bit value.
 *
 * INA226 registers are transmitted in big-endian format, while
 * many hosts are little-endian. This function swaps bytes accordingly.
 *
 * @param value 16-bit value in big-endian.
 * @return The 16-bit value in host-endian.
 */
static inline std::int16_t swap_bytes(std::int16_t value) {
    return static_cast<std::int16_t>(((value & 0xFF) << 8) | ((value >> 8) & 0xFF));
}

/**
 * @brief Initialize the INA226 sensor.
 *
 * Sets up I2C communication with the sensor, writes the configuration register,
 * and writes the calibration register. Afterward, the sensor will measure current
 * (and bus voltage) continuously.
 *
 * @return true if successful, false if an I2C or register write error occurs.
 */
bool INA226::initialize() {
    // Set up the I2C interface for the INA226 sensor.
    _i2c_fd = wiringPiI2CSetup(_INA226_I2C_ADDRESS);
    if (_i2c_fd < 0) {
        std::cerr << "Interface with address failed!" << std::endl;
        return false;
    }

    // Write the configuration register.
    if (wiringPiI2CWriteReg16(_i2c_fd, _CONFIG_REGISTER, _CONFIG_VALUE) < 0) {
        std::cerr << "write configuration register failed!" << std::endl;
        return false;
    }

    // Write the calibration register.
    if (wiringPiI2CWriteReg16(_i2c_fd, _CALIBRATION_REGISTER, _CALIBRATION_VALUE) < 0) {
        std::cerr << "write calibration register failed!" << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief Read the current from the INA226 in milliamps.
 *
 * Reads the 16-bit Current register (0x04), swaps the byte order,
 * and multiplies by _CURRENT_LSB to get mA. If the read fails,
 * returns signaling NaN.
 *
 * @return Current in mA, or NaN if an I2C read fails.
 */
float INA226::readCurrent_mA() {
    // Read the raw 16-bit value from the Current register.
    int raw = wiringPiI2CReadReg16(_i2c_fd, _CURRENT_REGISTER);
    if (raw < 0) {
        // Return NaN to indicate an error.
        return std::numeric_limits<float>::signaling_NaN();
    }

    // Swap from big-endian (INA226) to host-endian format.
    std::int16_t raw_current = swap_bytes(static_cast<std::int16_t>(raw));

    // Convert to milliamps: Current = raw_current * _CURRENT_LSB.
    float current_mA = static_cast<float>(raw_current) * _CURRENT_LSB;
    return current_mA;
}

/**
 * @brief Put the INA226 into Power-Down (Shutdown) mode.
 *
 * This reads the Configuration register, clears bits [2..0] to 000,
 * and writes back the result. This stops measurements and reduces power.
 *
 * @return true on success, false if an I2C error occurs.
 */
bool INA226::power_down()
{
// 1. Read the current config register value (16-bit).
int raw_config = wiringPiI2CReadReg16(_i2c_fd, _CONFIG_REGISTER);
if (raw_config < 0) {
    return false;
}
// 2. Convert from big-endian to host-endian.
std::int16_t config = swap_bytes(static_cast<std::int16_t>(raw_config));

// 3. Clear the MODE bits [2..0] => 000 for power-down.
config &= ~0b0000000000000111; // zero out bits 0..2

// 4. Swap back and write to the device.
std::int16_t write_val = swap_bytes(config);
if (wiringPiI2CWriteReg16(_i2c_fd, _CONFIG_REGISTER, write_val) < 0) return false;

return true;
}

/**
 * @brief Wake the INA226 from power-down mode into continuous Shunt+Bus mode (0b111).
 *
 * Reads the config register, sets bits [2..0] to 111, and writes it back.
 *
 * @return true on success, false if an I2C error occurs.
 */
bool INA226::wake_up()
{
// 1. Read the current config register value.
int raw_config = wiringPiI2CReadReg16(_i2c_fd, _CONFIG_REGISTER);
if (raw_config < 0) {
    return false;
}
// 2. Swap bytes to host format.
std::int16_t config = swap_bytes(static_cast<std::int16_t>(raw_config));

    // 3. Set the MODE bits [2..0] => 111 = continuous shunt + bus.
config &= ~0b0000000000000111;   // clear bits [2..0]
config |=  0b0000000000000111;   // set them to 111

// 4. Swap back to big-endian and write.
std::int16_t write_val = swap_bytes(config);
if (wiringPiI2CWriteReg16(_i2c_fd, _CONFIG_REGISTER, write_val) < 0) return false;

return true;
}

} // namespace SlidingGate
