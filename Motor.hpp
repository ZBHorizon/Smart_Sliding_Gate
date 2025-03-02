/**
 * @file Motor.hpp
 * @brief Declaration of the Motor class for controlling the sliding gate.
 */

#pragma once
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <list>
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

    /**
     * @brief gets aktivated if the light barrier sensor is triggered.
     */
    static void light_barrier_isr();

    static void move_to_starting_position(float starting_position);

    /**
     * @brief gets aktivated if the open end switch is triggered.
     */
    static void open_switch_isr();

    /**
     * @brief gets aktivated if the close end switch is triggered.
     */
    static void close_switch_isr();

    inline static std::condition_variable motor_cv; ///< Condition Variable for motor control

private:
    /**
     * @brief Contains parameters for timing and speed regulation.
     */
    struct _Param {
        static constexpr float _CALIBRATION_SPEED = 0.10f;  ///< Calibration speed
        static constexpr uint16_t _CURRENT_THRESHOLD = 5000;  ///< Overcurrent threshold (mA)
        static constexpr milliseconds _OVERCURRENT_DURATION = 100ms; ///< Duration to confirm overcurrent
        static constexpr milliseconds _FIRST_TIME_TO_OPEN = 26000ms;  //< Time to fully open (ms)
        static constexpr milliseconds _FIRST_TIME_TO_CLOSE = 26000ms; //< Time to fully close (ms)
        static constexpr int LIST_SIZE = 5; //< Maximum amount of items in list
    };
    // Internal static variables
    // Definitions for non-inline static members
    
    static std::list<milliseconds> append_time(std::list<milliseconds> list, milliseconds time);

    inline static std::condition_variable _open_switch_cv; ///< Condition Variable
    inline static std::condition_variable _close_switch_cv; ///< Condition Variable 
       
    inline static steady_clock::time_point _overcurrent_start = std::chrono::steady_clock::now();  ///< Overcurrent mechanism

    inline static float _actual_speed = 0.0f;      //< Current speed
    inline static float _actual_position = 0.0f;     //< Current position (percentage)
    inline static milliseconds _time_to_open = _Param::_FIRST_TIME_TO_OPEN;  //< Time to fully open (ms)
    inline static milliseconds _time_to_close = _Param::_FIRST_TIME_TO_CLOSE; //< Time to fully close (ms)

    inline static bool _is_time_to_open_calibrated = false;       //< Calibration flag
    inline static bool _is_time_to_close_calibrated = false;       //< Calibration flag

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
    inline static time_point<steady_clock>_slow_timestamp = std::chrono::steady_clock::now();    ///< Movement slow time
    inline static float _slow_speed = 0.0f;
    inline static time_point<steady_clock> _stop_timestamp = std::chrono::steady_clock::now();     ///< Movement end time
    inline static milliseconds _start_position_ms = 0ms;        ///< Starting position time

    inline static std::list<milliseconds> avarage_time_to_open;  // list with maximum 5 elemnts
    inline static std::list<milliseconds> avarage_time_to_close; // list with maximum 5 elemnts

    /**
     * @brief Updates the current gate position.
     */
    static void update_current_position();

    /**
     * @brief Checks for an overcurrent condition.
     */
    static void check_for_overcurrent();


    static void update_times();
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


    inline static bool open_switch_triggered = false;
    inline static bool close_switch_triggered = false;





};

} // namespace SlidingGate
