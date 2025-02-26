/**
 * @file Motor.hpp
 * @brief Declaration of the Motor class for controlling the sliding gate.
 */

#pragma once
#include <cstdint>
#include <chrono>
#include <condition_variable>
using namespace std::chrono;

namespace SlidingGate {

/**
 * @brief Controls the motor for the sliding gate.
 */
class Motor {
public:
    /**
     * @brief Main loop for controlling the motor.
     */
    static void motor_loop(); 

    /**
     * @brief Calibrates the open/close timing.
     */
    static void calibrate_timing();

    /**
     * @brief Reads the current motor speed.
     * @return current speed
     */
    static float read_speed();

    /**
     * @brief Reads the current motor position.
     * @return current position percentage
     */
    static float read_position();

    /**
     * @brief Checks if the motor is calibrated.
     * @return true if calibrated
     */
    static bool is_calibrated();

private:
    // Internal static variables
    // Definitions for non-inline static members
    inline static std::condition_variable motor_cv; ///< Condition Variable for motor control
    inline static std::condition_variable open_switch_cv; ///< Condition Variable
    inline static std::condition_variable close_switch_cv; ///< Condition Variable 
       
    inline static steady_clock::time_point overcurrent_start = std::chrono::steady_clock::now();  ///< Overcurrent mechanism

    inline static float _actual_speed = 0.0f;      ///< Current speed
    inline static float _actual_position = 0.0f;     ///< Current position (percentage)
    inline static milliseconds _time_to_open = 0ms;  ///< Time to fully open (ms)
    inline static milliseconds _time_to_close = 0ms; ///< Time to fully close (ms)
    inline static bool _is_calibrated = false;       ///< Calibration flag

    inline static bool _overcurrent_active = false;  ///< Initialization

    /**
     * @brief Motor states.
     */
    enum class MotorState {
        Opening,   ///< Gate is opening
        Closing,   ///< Gate is closing
        None       ///< No action
    };

    inline static MotorState _motor_state = MotorState::None; ///< Current motor state
    inline static time_point<steady_clock>_start_timestamp = std::chrono::steady_clock::now();    ///< Movement start time
    inline static time_point<steady_clock> _stop_timestamp = std::chrono::steady_clock::now();     ///< Movement end time
    inline static milliseconds _start_position_ms = 0ms;        ///< Starting position time

    /**
     * @brief Updates the current gate position.
     */
    static void update_current_position();

    /**
     * @brief Checks for an overcurrent condition.
     */
    static void check_for_overcurrent();

    /**
     * @brief Checks if the light barrier sensor is active.
     */
    static void light_barrier_isr();

    /**
     * @brief Checks if the end switches are activated.
     */
    static bool check_end_switches();

    /**
     * @brief Updates the motor state.
     */
    static void update_states();

    /**
     * @brief Sets the motor speed.
     * @param speed Target speed.
     */
    static void set_speed(float speed);

    /**
     * @brief Updates motor control based on the position.
     * @return true if an error state is detected.
     */
    static bool update_motor();

    // isr for end switches
    static void open_switch_isr();

    static void close_switch_isr();



    /**
     * @brief Contains parameters for timing and speed regulation.
     */
    struct Param {
        inline static float calibration_speed = 0.30f;  ///< Calibration speed
        static constexpr uint16_t current_threshold = 5000;  ///< Overcurrent threshold (mA)
        inline static milliseconds overcurrent_duration = 100ms; ///< Duration to confirm overcurrent
    };



};

} // namespace SlidingGate
