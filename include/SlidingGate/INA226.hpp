/**
 * @file INA226.hpp
 * @brief INA226 Current Sensor Class
 *
 * This header file declares the class for interacting with the INA226
 * high-side or low-side current and power monitor via I2C/SMBus.
 *
 * ----------------------------------------------------------------------------
 * IMPORTANT REGISTERS (Reference: INA226 Datasheet, Section 7)
 * ----------------------------------------------------------------------------
 * The INA226 has the following 16-bit registers (Table 7-1 in the datasheet):
 *
 * Register | Address | R/W  | Description
 * -------- | ------ | ---- | ------------------------------------------------------
 * CONFIG   | 0x00   | R/W  | Configuration: conversion times, averaging, mode, etc.
 * SHUNT_V  | 0x01   | R    | Measured shunt voltage (2.5 µV LSB, signed 16-bit)
 * BUS_V    | 0x02   | R    | Measured bus voltage (1.25 mV LSB, 16-bit)
 * POWER    | 0x03   | R    | Calculated power in watts (LSB = 25 * Current_LSB)
 * CURRENT  | 0x04   | R    | Calculated current (depends on Calibration Register)
 * CALIB    | 0x05   | R/W  | Calibration register (sets Current_LSB, etc.)
 * MASK/EN  | 0x06   | R/W  | Alert configuration, conversion-ready flag, etc.
 * ALERTLIM | 0x07   | R/W  | Alert limit register
 * MFR_ID   | 0xFE   | R    | Manufacturer ID
 * DIE_ID   | 0xFF   | R    | Die ID
 *
 * Note:
 *  - CURRENT (0x04) and POWER (0x03) remain zero until the CALIBRATION (0x05)
 *    register is programmed with an appropriate value.
 *
 * ----------------------------------------------------------------------------
 * CONFIGURATION REGISTER DETAILS (Register 0x00)
 * ----------------------------------------------------------------------------
 *
 * Bits (16 total):    [ RST | -- | -- | -- | AVG2 | AVG1 | AVG0 | VBUSCT2 | VBUSCT1 | VBUSCT0 | VSHCT2 | VSHCT1 | VSHCT0 | MODE3 | MODE2 | MODE1 ]
 *
 *  - RST (bit 15): Writing '1' resets all registers to default; self-clearing.
 *  - AVG (bits 11..9): Averaging Mode (Table 7-3). Determines the number of samples.
 *
 *    AVG: Averaging Mode  
 *    Bits 9–11 determine how many samples are collected and averaged.  
 *    Table 7-3. AVG Bit Settings [11:9]:  
 *      AVG2(D11) | AVG1(D10) | AVG0(D9) | Number of Averages  
 *      ----------|-----------|----------|---------------------
 *      0         | 0         | 0        | 1  
 *      0         | 0         | 1        | 4  
 *      0         | 1         | 0        | 16  
 *      0         | 1         | 1        | 64  
 *      1         | 0         | 0        | 128  
 *      1         | 0         | 1        | 256  
 *      1         | 1         | 0        | 512  
 *      1         | 1         | 1        | 1024  
 *    (Shaded or bold in the datasheet = default)
 *
 *  - VBUSCT (bits 8..6): Bus Voltage Conversion Time  
 *    Table 7-4. VBUSCT [8:6]  
 *      VBUSCT2(D8)|VBUSCT1(D7)|VBUSCT0(D6)| Conversion Time
 *      -----------|-----------|----------|----------------
 *      0          | 0         | 0        | 140 µs
 *      0          | 0         | 1        | 204 µs
 *      0          | 1         | 0        | 332 µs
 *      0          | 1         | 1        | 588 µs
 *      1          | 0         | 0        | 1.1 ms
 *      1          | 0         | 1        | 2.116 ms
 *      1          | 1         | 0        | 4.156 ms
 *      1          | 1         | 1        | 8.244 ms
 *
 *  - VSHCT (bits 5..3): Shunt Voltage Conversion Time  
 *    Table 7-5. VSHCT [5:3]
 *      VSHCT2(D5)|VSHCT1(D4)|VSHCT0(D3)| Conversion Time
 *      ----------|----------|---------|----------------
 *      0         | 0        | 0       | 140 µs
 *      0         | 0        | 1       | 204 µs
 *      0         | 1        | 0       | 332 µs
 *      0         | 1        | 1       | 588 µs
 *      1         | 0        | 0       | 1.1 ms
 *      1         | 0        | 1       | 2.116 ms
 *      1         | 1        | 0       | 4.156 ms
 *      1         | 1        | 1       | 8.244 ms
 *
 *  - MODE (bits 2..0): Operating Mode  
 *    Table 7-6. Mode Settings [2:0]
 *      MODE3(D2)|MODE2(D1)|MODE1(D0)| Mode
 *      ---------|---------|---------|-----------------------------
 *      0        | 0       | 0       | Power-Down (Shutdown)
 *      0        | 0       | 1       | Shunt Voltage, Triggered
 *      0        | 1       | 0       | Bus Voltage, Triggered
 *      0        | 1       | 1       | Shunt and Bus, Triggered
 *      1        | 0       | 0       | Power-Down (Shutdown)
 *      1        | 0       | 1       | Shunt Voltage, Continuous
 *      1        | 1       | 0       | Bus Voltage, Continuous
 *      1        | 1       | 1       | Shunt and Bus, Continuous (Default)
 *
 * ----------------------------------------------------------------------------
 * CURRENT REGISTER (0x04)
 * ----------------------------------------------------------------------------
 * 7.1.5 Current Register (04h) (Read-Only)
 * If averaging is enabled, this register displays the averaged value.
 * The value of the Current Register is calculated by multiplying the decimal value
 * in the Shunt Voltage Register with the decimal value of the Calibration Register,
 * according to Equation (3) in the datasheet.
 *
 * Table 7-10. Current Register (04h) (Read-Only)
 *  Bit#    D15   D14  D13  D12  D11  D10  D9   D8   D7   D6   D5   D4   D3   D2   D1   D0
 *  Name  | CSIGN | CD14| CD13| CD12| CD11| CD10| CD9| CD8| CD7| CD6| CD5| CD4| CD3| CD2| CD1| CD0
 *  Reset |   0   |  0  |  0  |  0  |  0  |  0  |  0 |  0 |  0 |  0 |  0 |  0 |  0 |  0 |  0 |  0
 *
 * ----------------------------------------------------------------------------
 * CALIBRATION REGISTER (0x05)
 * ----------------------------------------------------------------------------
 * 7.1.6 Calibration Register (05h) (Read/Write)
 * This register provides the device with the value of the shunt resistor,
 * sets the resolution of the Current Register, and also determines the power LSB.
 * See "Programming the Calibration Register" in the datasheet.
 *
 * Table 7-11. Calibration Register (05h) (Read/Write)
 *   Bit#   D15  D14  D13  D12  D11  D10  D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *   Name  | -- | FS14| FS13| FS12| FS11| FS10|FS9|FS8|FS7|FS6|FS5|FS4|FS3|FS2|FS1|FS0
 *   Reset |  0 |  0  |  0  |  0  |  0  |  0  | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0
 *
 * ----------------------------------------------------------------------------
 * NOTE ON CALIBRATION AND _CURRENT_LSB:
 * - Calibration = (0.00512) / (Current_LSB * Rshunt)
 * - Then the Current Register (0x04) stores: (ShuntVoltage * Calibration) / 2048
 *
 * ----------------------------------------------------------------------------
 * Example default usage in code:
 *   - _CONFIG_VALUE = 0x4127 => Continuous Shunt+Bus, ~1.1 ms each, no averaging
 *   - _CALIBRATION_VALUE = 2560 => suitable for 2mΩ, ~15A, 1mA/bit
 *   - _CURRENT_LSB = 1.0f => (1 mA/bit)
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
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/

public:

  /**
         * @brief Initializes and configures the sensor.
         * @return True if the sensor was successfully initialized
         */
  static bool initialize();

  /**
         * @brief Wakes up the INA226, e.g., Continuous Shunt+Bus
         * 
         * @return true if the register was successfully written
         *         false if an I2C error occurred
         */
  static bool wake_up();

  /**
         * @brief Puts the INA226 into Power-Down / Shutdown Mode.
         * 
         * @return true if the register was successfully written
         *         false if an I2C error occurred
         */
  static bool  power_down();
  /**
         * @brief Reads the current from the sensor in milliamps (mA).
         * @return The current reading in mA. and NAN if error
         */
  static float readCurrent_mA();

  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/

private:

  inline static int             _i2c_fd               = -1; //!< I2C file descriptor for the INA226 device.
  //define INA226 register addresses and constant values
  static constexpr std::int32_t _INA226_I2C_ADDRESS   = 0x40; //!< I2C address
  static constexpr std::int32_t _CONFIG_REGISTER      = 0x00; //!< Configuration register address
  static constexpr std::int32_t _CURRENT_REGISTER     = 0x04; //!< Current register address
  static constexpr std::int32_t _CALIBRATION_REGISTER = 0x05; //!< Calibration register address

  static constexpr std::int8_t  _CONFIG_AVG    = 0b000;
  static constexpr std::int8_t  _CONFIG_VBUSCT = 0b100;
  static constexpr std::int8_t  _CONFIG_VSHCT  = 0b100;
  static constexpr std::int8_t  _CONFIG_MODE   = 0b111;
  static constexpr std::int16_t _CONFIG_VALUE  = (_CONFIG_AVG << 9) | (_CONFIG_VBUSCT << 6) | (_CONFIG_VSHCT << 3) | _CONFIG_MODE; //! configuration

  static constexpr std::int16_t _CALIBRATION_VALUE     = 2'560; //!< Calibration value for 1 mA/bit with 2 mΩ shunt resistor.
  static constexpr float        _CURRENT_LSB           = 1.0f;  //!< Current LSB in mA/bit.
  static constexpr float        _DESIRED_MAX_CURRENT_A = 15.0;
};

} // namespace SlidingGate
