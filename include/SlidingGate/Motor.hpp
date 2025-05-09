/**
 * @file Motor.hpp
 * @brief Declaration of the Motor class which controls the sliding gate mechanism.
 */

#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <list>
using namespace std::chrono;

namespace SlidingGate {

/**
 * @brief Controls the motor for the sliding gate.
 */
class Motor {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/

public:

  /**
     * @brief Main control loop for the motor.
     * 
     * Continuously handles motor jobs and updates motor position and speed.
     */
  static void motor_loop();

  /**
     * @brief Retrieves the current speed of the motor.
     * @return Current motor speed.
     */
  static float read_speed();

  /**
     * @brief Retrieves the current gate position.
     * @return Gate position as a percentage value (0.0 to 1.0).
     */
  static float read_position();

  /**
     * @brief Checks if the motor has been calibrated.
     * @return true if both open and close calibration procedures have completed.
     */
  static bool is_calibrated();

  /**
     * @brief Interrupt service routine triggered by the light barrier sensor.
     */
  static void light_barrier_isr();

  /**
     * @brief Moves the gate to the designated starting position for calibration.
     * 
     * @param starting_position The target starting position (0.0 for closed, 1.0 for open).
     */
  static void move_to_starting_position(float starting_position);

  /**
     * @brief Interrupt service routine triggered by the open end switch.
     */
  static void open_switch_isr();

  /**
     * @brief Interrupt service routine triggered by the close end switch.
     */
  static void close_switch_isr();

  inline static std::condition_variable motor_cv; ///< Condition Variable for motor control


  inline static milliseconds get_time_to_open() { return _time_to_open; }   ///< Getter for time to open
  inline static milliseconds get_time_to_close() { return _time_to_close; } ///< Getter for time to close

  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/

private:

  /**
     * @brief Contains parameters for timing and speed regulation.
     */
  struct _Param {
    static constexpr float        _CALIBRATION_SPEED    = 0.10f;    ///< Calibration speed
    static constexpr uint16_t     _CURRENT_THRESHOLD    = 5'000;    ///< Overcurrent threshold (mA)
    static constexpr milliseconds _OVERCURRENT_DURATION = 100ms;    ///< Duration to confirm overcurrent
    static constexpr milliseconds _FIRST_TIME_TO_OPEN   = 26'000ms; //< Time to fully open (ms)
    static constexpr milliseconds _FIRST_TIME_TO_CLOSE  = 26'000ms; //< Time to fully close (ms)
    static constexpr int          LIST_SIZE             = 5;        //< Maximum amount of items in list
  };
  // Internal static variables
  // Definitions for non-inline static members

  static std::list<milliseconds> append_time(std::list<milliseconds> list, milliseconds time);

  inline static std::condition_variable _open_switch_cv;  ///< Condition Variable
  inline static std::condition_variable _close_switch_cv; ///< Condition Variable

  inline static steady_clock::time_point _overcurrent_start = std::chrono::steady_clock::now(); ///< Overcurrent mechanism

  inline static float        _actual_speed    = 0.0f;                         //< Current speed
  inline static float        _actual_position = 0.0f;                         //< Current position (percentage)
  inline static float        _start_position  = 0.0f;                         //< Start position (percentage)
  inline static milliseconds _time_to_open    = _Param::_FIRST_TIME_TO_OPEN;  //< Time to fully open (ms)
  inline static milliseconds _time_to_close   = _Param::_FIRST_TIME_TO_CLOSE; //< Time to fully close (ms)

  inline static bool _is_time_to_open_calibrated  = false; //< Calibration flag
  inline static bool _is_time_to_close_calibrated = false; //< Calibration flag

  inline static bool _overcurrent_active = false; ///< Initialization

  static std::thread time_thread; ///< Thread for time measurement

  /**
     * @brief Motor states.
     */
  enum class MotorState {
    Opening, ///< Gate is opening
    Closing, ///< Gate is closing
    None     ///< No action
  };

  inline static MotorState               _motor_state       = MotorState::None;                 ///< Current motor state
  inline static time_point<steady_clock> _start_timestamp   = std::chrono::steady_clock::now(); ///< Movement start time
  inline static time_point<steady_clock> _slow_timestamp    = std::chrono::steady_clock::now(); ///< Movement slow time
  inline static float                    _slow_speed        = 0.0f;
  inline static time_point<steady_clock> _stop_timestamp    = std::chrono::steady_clock::now(); ///< Movement end time
  inline static milliseconds             _start_position_ms = 0ms;                              ///< Starting position time

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


  inline static bool open_switch_triggered  = false;
  inline static bool close_switch_triggered = false;

  inline static constexpr const char* _YELLOW = "\033[33m";
  inline static constexpr const char* _RESET  = "\033[0m";
};

} // namespace SlidingGate
