#include <Initialize.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

// Bring selected symbols into the global scope (optional, based on your coding guidelines)
using std::runtime_error;
using std::lock_guard;

//========================================================================
// INA226 Class Implementation
//========================================================================

namespace SlidingGate {

    uint16_t INA226::read16(uint8_t reg_address) {
        lock_guard<std::mutex> lock(i2c_mutex);
        uint16_t result = static_cast<uint16_t>(wiringPiI2CReadReg16(i2c_fd, reg_address));
        // Swap bytes for endian conversion.
        return (result << 8) | (result >> 8);
    }

    void INA226::write16(uint8_t reg_address, uint16_t value) {
        lock_guard<std::mutex> lock(i2c_mutex);
        wiringPiI2CWriteReg16(i2c_fd, reg_address, (value << 8) | (value >> 8));
    }

    void INA226::initialize() {
        // Setup I2C communication with INA226.
        i2c_fd = wiringPiI2CSetup(I2C_ADDRESS);
        if (i2c_fd < 0) {
            throw runtime_error("INA226: Device not found on I2C bus!");
        }

        // Calculate current_lsb based on the maximum current and 15-bit resolution.
        current_lsb = MAX_CURRENT / static_cast<float>(1 << 15);
        float calibration = 0.00512f / (current_lsb * R_SHUNT);
        uint16_t calib_reg = static_cast<uint16_t>(std::floor(calibration));
        // Recalculate current_lsb with the calibration register value.
        current_lsb = 0.00512f / (R_SHUNT * calib_reg);
        write16(0x05, calib_reg); // INA226 calibration register address is 0x05.
    }

    int16_t INA226::read_current() {
        // INA226 current register address is 0x04.
        int16_t current_reg = static_cast<int16_t>(read16(0x04));
        // Multiply by current_lsb and convert A to mA.
        float current_mA = current_reg * current_lsb * 1000.0f;
        return static_cast<int16_t>(current_mA);
    }

    //========================================================================
    // Pin::Manager Implementation
    //========================================================================

    void Pin::Manager::initialize_gpio() {
        if (wiringPiSetup() == -1) {
            throw runtime_error("Failed to initialize wiringPi!");
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
    }

} // namespace SlidingGate
