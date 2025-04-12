/**
 * @file Motor.cpp
 * @brief Implementation of the Motor class for controlling the sliding gate.
 */
#include <SlidingGate/INA226.hpp>
#include <SlidingGate/IO.hpp>
#include <SlidingGate/Initialize.hpp>
#include <SlidingGate/Log.hpp>
#include <SlidingGate/Motor.hpp>
#include <SlidingGate/job.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib> // for std::abs
#include <fstream>
#include <iostream>
#include <list>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono;

namespace SlidingGate {
//! Mutex to protect all static motor data.
static std::mutex motor_mutex;

milliseconds average(std::list<milliseconds> list) {
  milliseconds sum  = 0ms;
  sum              -= *std::minmax_element(list.begin(), list.end()).first;
  return sum / list.size();
}

std::list<milliseconds> Motor::append_time(std::list<milliseconds> list, milliseconds time) {
  if (list.size() >= _Param::LIST_SIZE) { list.pop_front(); }
  list.push_back(time);
  return list;
}


/**
 * @brief Updates the internal state of the motor based on the current speed.
 */
void Motor::update_states() {
  if (_actual_speed == 0) {
    if (_motor_state != MotorState::None) {
      if (_is_time_to_open_calibrated) _stop_timestamp = steady_clock::now();
      _motor_state = MotorState::None;
      LOG_INFO() << "Motorstate: " << _YELLOW << "Motorstate::None" << _RESET << ".";
    }
  } else if (_actual_speed < 0 && _motor_state != MotorState::Closing) {
    _motor_state       = MotorState::Closing;
    _start_timestamp   = steady_clock::now();
    _start_position_ms = duration_cast<milliseconds>(_time_to_close * _actual_position);
    _start_position    = _actual_position;
    LOG_INFO() << "Motorstate: " << _YELLOW << "Motorstate::Closing" << _RESET << ".";
  } else if (_actual_speed > 0 && _motor_state != MotorState::Opening) {
    _motor_state       = MotorState::Opening;
    _start_timestamp   = steady_clock::now();
    _start_position_ms = duration_cast<milliseconds>(_time_to_open * _actual_position);
    _start_position    = _actual_position;
    LOG_INFO() << "Motorstate: " << _YELLOW << "Motorstate::Opening" << _RESET << ".";
  }
}

/**
 * @brief Checks if any of the end switches are activated.
 */
bool Motor::check_end_switches() {
  if ((_motor_state == MotorState::Opening && !IO::digitalRead(Pin::OPEN_SWITCH))
      || (_motor_state == MotorState::Closing && !IO::digitalRead(Pin::CLOSE_SWITCH))
      || (_motor_state == MotorState::Closing && !IO::digitalRead(Pin::LIGHT_BARRIER))) {
    LOG_INFO() << "check_end_switches() : " << _YELLOW << "true" << _RESET << ".";
    return true;
  }
  return false;
}

/**
 * @brief Checks if the light barrier sensor is active.
 */
void Motor::light_barrier_isr() {
  std::cout << "ligt barrier" << std::endl;
  // Check if light barrier active when closing
  if (_motor_state == MotorState::Closing) {
    LOG_INFO() << "Light Barrier: " << _YELLOW << "Interruptet" << _RESET << " new job to " << _YELLOW << "100 %" << _RESET << ".";
    set_speed(0.0f);
    job::create_job(1.0f);
  }
}

/**
 * @brief Checks for an overcurrent condition and takes appropriate action.
 */
void Motor::check_for_overcurrent() {
  if (INA226::readCurrent_mA() > _Param::_CURRENT_THRESHOLD) {
    //if motor is after acceleration phase then stop instantly
    if (std::abs(_actual_speed) >= 0.95f) { _overcurrent_active = true; }
    //otherwise check if overcurrent is active for a certain duration
    auto now = steady_clock::now();
    if (!_overcurrent_active) {
      _overcurrent_active = true;
      _overcurrent_start  = now;
    } else if (duration_cast<milliseconds>(now - _overcurrent_start) >= _Param::_OVERCURRENT_DURATION) {
      _overcurrent_active = false;
    }
  } else {
    _overcurrent_active = false;
  }
  if (_overcurrent_active) {
    if (_motor_state != MotorState::Closing) {
      set_speed(0.0f);
      job::create_job(1.0f);
    } else {
      set_speed(0.0f);
      job::delete_job();
    }
  }
}

/**
 * @brief Updates the current position of the gate.
 * 
 * Calculates the gate's current position based on elapsed time since motion start,
 * current speed, and the full open/close duration. Logs the new position and the
 * position change rate in percentage per second.
 */
void Motor::update_current_position() {
  using namespace std::chrono;
  steady_clock::time_point now             = steady_clock::now();
  milliseconds             elapsed_ms      = duration_cast<milliseconds>(now - _start_timestamp);
  const float              elapsed_seconds = static_cast<float>(elapsed_ms.count()) / 1000.0f;
  const float              progress        = _actual_speed * elapsed_seconds;

  if (_motor_state == MotorState::Opening) {
    const float total_time_open = static_cast<float>(_time_to_open.count()) / 1000.0f;
    float       new_position    = _start_position + (progress / total_time_open);
    if (!IO::digitalRead(Pin::OPEN_SWITCH)) new_position = 1.0f;
    _actual_position = std::min(1.0f, new_position);
  } else if (_motor_state == MotorState::Closing) {
    const float total_time_close = static_cast<float>(_time_to_close.count()) / 1000.0f;
    float       new_position     = _start_position - (progress / total_time_close);
    if (!IO::digitalRead(Pin::CLOSE_SWITCH)) new_position = 0.0f;
    _actual_position = std::max(0.0f, new_position);
  }
  LOG_INFO() << "Current Motor Position: " << _YELLOW << (_actual_position * 100.0f) << " % " << _RESET << ".";

  // Calculate and log position change rate in % per second.
  float position_change_rate = 0.0f;
  if (elapsed_seconds > 0.0f) { position_change_rate = ((_actual_position - _start_position) / elapsed_seconds) * 100.0f; }
  LOG_INFO() << "Positionchange in % per second: " << _YELLOW << position_change_rate << " % " << _RESET << ".";
}

/**
 * @brief Moves the gate to the specified starting position.
 * 
 * Initiates a calibration move by driving the gate either fully open (100%) or fully closed (0%)
 * and waiting for the appropriate end switch to trigger.
 * 
 * @param starting_position Target starting position; must be 1.0 for open or 0.0 for closed.
 */
void Motor::move_to_starting_position(float starting_position) {
  if (starting_position == 1.0f) {
    LOG_INFO() << "open with : " << _YELLOW << -_Param::_CALIBRATION_SPEED * 100.0f << _RESET << " % calibration speed.";
    std::unique_lock<std::mutex> lock(motor_mutex);
    set_speed(-_Param::_CALIBRATION_SPEED);
    _open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
    open_switch_triggered = false; // reset flag after wake-up
    return;
  }
  if (starting_position == 0.0f) {
    LOG_INFO() << "closing with : " << _YELLOW << _Param::_CALIBRATION_SPEED * 100.0f << _RESET << " % calibration speed.";
    std::unique_lock<std::mutex> lock(motor_mutex);
    set_speed(_Param::_CALIBRATION_SPEED);
    _close_switch_cv.wait(lock, [&] { return close_switch_triggered; });
    close_switch_triggered = false; // reset flag after wake-up
    return;
  }
  LOG_ERROR() << "position needs to be: " << _YELLOW << "0%" << _RESET << " or " << _YELLOW << "100%" << _RESET << "Starting position is: " << _YELLOW
              << starting_position * 100.0f << _RESET << ".";
}

/**
 * @brief Updates the time measurements used for calibration.
 * 
 * Measures the durations of gate movement, computes an average over recent measurements,
 * and updates the full open/close time values accordingly.
 */
void Motor::update_times() {
  LOG_INFO() << "Updated Time to Open: " << _YELLOW << _time_to_open << _RESET << " . Updated Time to Close: " << _YELLOW << _time_to_close << _RESET
             << ".";
  static std::mutex update_times_mutex;
  if (IO::digitalRead(Pin::CLOSE_SWITCH)) {
    if (!_is_time_to_open_calibrated) {
      std::cout << "measure time to open" << std::endl;
      std::unique_lock<std::mutex> lock(update_times_mutex);
      _open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
      open_switch_triggered      = false; // reset flag after wake-up
      milliseconds fast_duration = duration_cast<milliseconds>(_slow_timestamp - _start_timestamp);
      milliseconds slow_duration = duration_cast<milliseconds>((_stop_timestamp - _slow_timestamp) / _slow_speed);
      _time_to_open              = fast_duration + slow_duration;
      avarage_time_to_open.push_back(_time_to_open);
      _slow_speed                 = 0.0f;
      _is_time_to_open_calibrated = true;
      return;
    }
    std::unique_lock<std::mutex> lock(update_times_mutex);
    _open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
    open_switch_triggered = false; // reset flag after wake-up
    if (IO::digitalRead(Pin::OPEN_SWITCH)) {
      milliseconds open_duration = duration_cast<milliseconds>(_stop_timestamp - _start_timestamp);
      avarage_time_to_open       = append_time(avarage_time_to_open, open_duration);
      _time_to_open              = average(avarage_time_to_open); //return the avrage value of all elemts in list
    }
    return;
  }
  if (IO::digitalRead(Pin::OPEN_SWITCH)) {
    if (!_is_time_to_close_calibrated) {
      std::cout << "measure time to close" << std::endl;
      std::unique_lock<std::mutex> lock(update_times_mutex);
      _close_switch_cv.wait(lock, [&] { return close_switch_triggered; });
      close_switch_triggered     = false; // reset flag after wake-up
      milliseconds fast_duration = duration_cast<milliseconds>(_slow_timestamp - _start_timestamp);
      milliseconds slow_duration = duration_cast<milliseconds>((_stop_timestamp - _slow_timestamp) / _slow_speed);
      _time_to_close             = fast_duration + slow_duration;
      avarage_time_to_close.push_back(_time_to_close);
      _slow_speed                  = 0.0f;
      _is_time_to_close_calibrated = true;
      return;
    }
    std::unique_lock<std::mutex> lock(update_times_mutex);
    _close_switch_cv.wait(lock, [&] { return close_switch_triggered; });
    close_switch_triggered = false; // reset flag after wake-up
    if (IO::digitalRead(Pin::CLOSE_SWITCH)) {
      milliseconds close_duration = duration_cast<milliseconds>(_stop_timestamp - _start_timestamp);
      avarage_time_to_close       = append_time(avarage_time_to_close, close_duration);
      _time_to_close              = average(avarage_time_to_close); //return the avrage value of all elemts in list
    }
    return;
  }
}


/**
 * @brief Returns the current gate position.
 * @return Current position percentage.
 */
float Motor::read_position() { return _actual_position; }

/**
 * @brief Main motor control loop which waits for jobs and updates motor control.
 */
void Motor::motor_loop() {
  while (true) {
    // condition variable wait: schlafe, bis job::ready true ist
    std::unique_lock<std::mutex> lock(motor_mutex);
    if (!job::is_job_active()) {
      LOG_INFO() << "Motor : " << _YELLOW << "sleeping" << _RESET << " Waiting for wake up.";
      motor_cv.wait(lock, [] { return job::ready; });
      std::thread time_thread(&Motor::update_times);
      time_thread.detach();
      job::ready = false;
      LOG_INFO() << "Motor : " << _YELLOW << "wake up" << _RESET << " .";
    }
    // ... weiterer Code im Loop

    //check_for_overcurrent();

    if (!update_motor()) {
      job::delete_job();
      continue;
    }
    // sleep für 1ms (davor muss der mutex unlocken und danach wieder locken)
    lock.unlock();
    std::this_thread::sleep_for(50ms);
  }
}

/**
 * @brief Updates the motor controls by adjusting speed based on keyframe interpolation.
 * @return true if an error state is detected.
 */
bool Motor::update_motor() {
  update_current_position();
  float speed = job::get_speed();
  LOG_INFO() << "Motor job get speed : " << _YELLOW << speed * 100.0f << " % " << _RESET << ".";
  if (std::isnan(speed)) {
    set_speed(0.0f);
    return false;
  }
  update_states();
  if (check_end_switches()) {
    set_speed(0.0f);
    LOG_INFO() << _YELLOW << "Motor end switch reached. " << _RESET << "Stopping motor.";
    return false;
  }
  set_speed(speed);
  return true;
}

/**
 * \brief Sets the motor speed and updates its state.
 * \param speed Target speed.
 * 
 * This function also handles speed changes mid-motion by resetting the baseline start time and position,
 * ensuring that the position calculation remains accurate even when the speed is adjusted during motion.
 */
void Motor::set_speed(float speed) {
  LOG_INFO() << "Motor set_speed : " << _YELLOW << speed * 100.0f << " % " << _RESET << ".";
  // If motor is already in motion and speed is changing, update the baseline.
  if (_motor_state != MotorState::None && std::abs(speed - _actual_speed) > 0.001f) {
    _start_timestamp = std::chrono::steady_clock::now();
    _start_position  = _actual_position;
  }
  if (IO::digitalRead(Pin::DIRECTION) != (speed >= 0 ? LOW : HIGH)) { IO::digitalWrite(Pin::DIRECTION, (speed >= 0 ? LOW : HIGH)); }
  uint8_t int_speed = static_cast<uint8_t>(std::abs(speed) * 128);
  IO::pwmWrite(Pin::PWM, int_speed);
  _actual_speed = speed;
  update_states();
}

/**
 * @brief Returns the current motor speed.
 * @return Current speed.
 */
float Motor::read_speed() { return _actual_speed; }

/**
 * @brief Checks whether the motor is calibrated.
 * @return true if calibrated.
 */
bool Motor::is_calibrated() {
  std::unique_lock<std::mutex> lock(motor_mutex);
  return _is_time_to_open_calibrated && _is_time_to_close_calibrated;
}

// isr for end switches
void Motor::open_switch_isr() {
  std::cout << "open_switch_isr()" << std::endl;
  if ((_motor_state == MotorState::Closing)) return;
  set_speed(0.0f);
  job::delete_job();
  if (!_is_time_to_open_calibrated) return;
  _stop_timestamp       = steady_clock::now();
  open_switch_triggered = true;
  _close_switch_cv.notify_all();
};

void Motor::close_switch_isr() {
  std::cout << "close_switch_isr()" << std::endl;
  if (_motor_state == MotorState::Opening) return;
  set_speed(0.0f);
  job::delete_job();
  if (!_is_time_to_close_calibrated) return;
  _stop_timestamp        = steady_clock::now();
  close_switch_triggered = true;
  _close_switch_cv.notify_all();
};

} // namespace SlidingGate
