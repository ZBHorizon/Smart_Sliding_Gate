/**
 * @file Motor.cpp
 * @brief Implementierung der Motor-Klasse.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cstdint>
#include <cstdlib>    // for std::abs
#include <mutex>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <Motor.hpp>
#include <condition_variable>
#include <list>
#include <algorithm>
#include <atomic>

// WiringPi includes (Raspberry Pi)
#include <wiringPi.h>
#include <Initialize.hpp>
#include <INA226.hpp>
#include <job.hpp>

using namespace std::chrono;

namespace SlidingGate {
//! Mutex to protect all static motor data.
static std::mutex motor_mutex;
static bool open_switch_triggered = false;
static bool close_switch_triggered = false;



/**
 * @brief Updates the internal state of the motor based on the current speed.
 */
void Motor::update_states() {
    if (_actual_speed == 0) {  
        if (_motor_state != MotorState::None) {
            _stop_timestamp = steady_clock::now();
            _motor_state = MotorState::None;
        }
    }
    else if (_actual_speed < 0 && _motor_state != MotorState::Closing) {
        _motor_state = MotorState::Closing;
        _start_timestamp = steady_clock::now();
        _start_position_ms = duration_cast<milliseconds>(_time_to_close * _actual_position);
    }
    else if (_actual_speed > 0 && _motor_state != MotorState::Opening) {
        _motor_state = MotorState::Opening;
        _start_timestamp = steady_clock::now();
        _start_position_ms = duration_cast<milliseconds>(_time_to_open * _actual_position);
    }
}

/**
 * @brief Checks if any of the end switches are activated.
 */
bool Motor::check_end_switches() {
    if ((_motor_state == MotorState::Opening && digitalRead(Pin::OPEN_SWITCH)) ||
        (_motor_state == MotorState::Closing && digitalRead(Pin::CLOSE_SWITCH)) || 
        (_motor_state == MotorState::Closing && digitalRead(Pin::LIGHT_BARRIER))) {
            return true;
    }
    return false;
}

/**
 * @brief Checks if the light barrier sensor is active.
 */
void Motor::light_barrier_isr() {
    // Check if light barrier active when closing
    if (_motor_state == MotorState::Closing) { 
        set_speed(0.0f);
        job::keyframe Open { 
            .speed = 0.0f,
            .position = 1.0f
        };
        job::create_job(Open);
    }
    
}

/**
 * @brief Checks for an overcurrent condition and takes appropriate action.
 */
void Motor::check_for_overcurrent(){
    if (INA226::readCurrent_mA() > Param::current_threshold) {
        //if motor is after acceleration phase then stop instantly
        if (std::abs(_actual_speed) >= 0.95f) {
            _overcurrent_active = true;
        }
        //otherwise check if overcurrent is active for a certain duration
        time_point now = steady_clock::now();
        if (!_overcurrent_active) {
            _overcurrent_active = true;
            overcurrent_start = now;
        } else if (duration_cast<milliseconds>(now - overcurrent_start) >= Param::overcurrent_duration) {
            _overcurrent_active = false;
        }
    } else {
        _overcurrent_active = false;
    }
    if (_overcurrent_active) {
        if (_motor_state != MotorState::Closing) {
        set_speed(0.0f);
        job::keyframe Open { 
            .speed = 0.0f,
            .position = 1.0f
        };
        job::create_job(Open);
        } else {
            set_speed(0.0f);
            job::delete_job();
        }
    }
}

/*!
 * \brief Updates the current position of the gate.
 */
void Motor::update_current_position() {
    // Calculate how the gate is been moving
    time_point now = steady_clock::now();
    milliseconds _current_position_ms = duration_cast<milliseconds>(now - _start_timestamp) + _start_position_ms;

    // Calculate current position in percentage 
    if (_motor_state == MotorState::Opening) {
        if (digitalRead(Pin::OPEN_SWITCH)) {
            _actual_position = 1.0f;
        } else {
            float position = (static_cast<float>(_current_position_ms.count()) * 100.0f) / 
                             static_cast<float>(_time_to_open.count());
            _actual_position = (position < 1.0f) ? position : 0.95f;
        }
    }else if (_motor_state == MotorState::Closing) {
        if (digitalRead(Pin::CLOSE_SWITCH)) {
            _actual_position = 0.0f;
        } else {
            float position = 100.0f - (static_cast<float>(_current_position_ms.count()) * 100.0f) / 
                             static_cast<float>(_time_to_close.count());
            _actual_position = (position > 0.0f) ? position : 0.05f;
        }
    }
}

/**
 * @brief Returns the current gate position.
 * @return Current position percentage.
 */
float Motor::read_position(){
    return _actual_position;
}

/**
 * @brief Main motor control loop which waits for jobs and updates motor control.
 */
void Motor::motor_loop() {
    wiringPiISR(Pin::LIGHT_BARRIER, INT_EDGE_RISING, light_barrier_isr);
    wiringPiISR(Pin::CLOSE_SWITCH, INT_EDGE_RISING, close_switch_isr);
    wiringPiISR(Pin::OPEN_SWITCH, INT_EDGE_RISING, open_switch_isr);
    while (true) {
        //condition variable wait check if job set. muss solange schlafen bis ein neuer job gesetzt wurde wenn kein job mehr anliegt
        std::unique_lock<std::mutex> lock(motor_mutex);
		if (!job::is_job_active()) {
            motor_cv.wait(lock, [&] { return job::ready; });
            job::ready = false;
		}

        //check_for_overcurrent();

        if (update_motor()) {
            job::delete_job();
            continue;
        } 
        // sleep für 1ms (davor muss der mutex unlocken und danach wieder locken)
        lock.unlock();
        std::this_thread::sleep_for(10ms);
    }
}

/**
 * @brief Updates the motor controls by adjusting speed based on keyframe interpolation.
 * @return true if an error state is detected.
 */
bool Motor::update_motor() {
    update_current_position();
    float speed = job::get_speed(_actual_position);
    if (std::isnan(speed)) {
        return true;
    }
    set_speed(speed);
    return false;
}

/**
 * @brief Sets the motor speed and updates its state.
 * @param speed Target speed.
 */
void Motor::set_speed(float speed){
    if (check_end_switches()) speed=0.0f;
    digitalWrite(Pin::DIRECTION, (speed >= 0 ? LOW : HIGH));
    //set speed 0 - 128 (0-100%)
    uint8_t int_speed = static_cast<uint8_t>(std::abs(speed) * 128);
    pwmWrite(Pin::PWM, int_speed);
    _actual_speed = speed;
    update_states();
}

/**
 * @brief Returns the current motor speed.
 * @return Current speed.
 */
float Motor::read_speed(){
    return _actual_speed;
}

/**
 * @brief Checks whether the motor is calibrated.
 * @return true if calibrated.
 */
bool Motor::is_calibrated() {
    std::unique_lock<std::mutex> lock(motor_mutex);
    return _is_calibrated;
}

// isr for end switches
void Motor::open_switch_isr(){
    set_speed(0.0f);
    job::delete_job();
    if (_is_calibrated) return;
    { 
        std::unique_lock<std::mutex> lock(motor_mutex);
        open_switch_triggered = true;
    }
    open_switch_cv.notify_all();
};

void Motor::close_switch_isr(){
    set_speed(0.0f);
    job::delete_job();
    if (_is_calibrated) return;
    { 
        std::unique_lock<std::mutex> lock(motor_mutex);
        close_switch_triggered = true;
    }
    close_switch_cv.notify_all();
};


/**
 * @brief Performs timing calibration by moving the gate and measuring times.
 *
 * Moves the gate from closed to open and vice versa to determine timing parameters.
 * Throws a runtime error if calibration times out.
 */
void Motor::calibrate_timing() {
    auto overall_start = steady_clock::now();
    {
        std::unique_lock<std::mutex> lock(motor_mutex);
        _is_calibrated = false;
        _time_to_open = 0ms;
        _time_to_close = 0ms;
    }
    enum CalibrationStep { move_to_starting_position, check_position, measure_time_to_fully_open, measure_time_to_fully_close };
    CalibrationStep calibration_step = check_position;

    while (true) {

        switch (calibration_step) {
            case check_position: {
                if (digitalRead(Pin::OPEN_SWITCH))
                    calibration_step = measure_time_to_fully_open;
                else if (digitalRead(Pin::CLOSE_SWITCH))
                    calibration_step = measure_time_to_fully_close;
                else
                    calibration_step = move_to_starting_position;
            } break;
            case move_to_starting_position: {
                std::unique_lock<std::mutex> lock(motor_mutex);
                set_speed(-Param::calibration_speed); 
                wiringPiISR(Pin::OPEN_SWITCH, INT_EDGE_RISING, open_switch_isr);
                open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
                open_switch_triggered = false; // reset flag after wake-up
                calibration_step = check_position;                    
            } break;
            case measure_time_to_fully_open: {
                set_speed(Param::calibration_speed);
                
                std::unique_lock<std::mutex> lock(motor_mutex);
                wiringPiISR(Pin::OPEN_SWITCH, INT_EDGE_RISING, open_switch_isr);
                auto start = steady_clock::now();

                open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
                open_switch_triggered = false; // reset flag after wake-up

                auto end = steady_clock::now();
                {
                    std::unique_lock<std::mutex> lock(motor_mutex);
                    _time_to_open = duration_cast<milliseconds>(end - start);
                }
                {
                    std::unique_lock<std::mutex> lock(motor_mutex);
                    if (_time_to_close.count() != 0) {
                        float factor = Param::calibration_speed; // Division durch 1.0f unnötig
                        _time_to_open = duration_cast<milliseconds>(_time_to_open * factor);
                        _time_to_close = duration_cast<milliseconds>(_time_to_close * factor);
                        _is_calibrated = true;
                        return;
                    }
                }
                calibration_step = measure_time_to_fully_close;
            } break;
            case measure_time_to_fully_close: {
                set_speed(-Param::calibration_speed);
                std::unique_lock<std::mutex> lock(motor_mutex);
                
                wiringPiISR(Pin::CLOSE_SWITCH, INT_EDGE_RISING, close_switch_isr);
                auto start = steady_clock::now();
                close_switch_cv.wait(lock, [&] { return close_switch_triggered; });
                close_switch_triggered = false; // reset flag after wake-up

                auto end = steady_clock::now();
                {
                    std::unique_lock<std::mutex> lock(motor_mutex);
                    _time_to_close = duration_cast<milliseconds>(end - start);
                    if (_time_to_open.count() != 0) {
                        float factor = Param::calibration_speed;
                        _time_to_open = duration_cast<milliseconds>(_time_to_open * factor);
                        _time_to_close = duration_cast<milliseconds>(_time_to_close * factor);
                        _is_calibrated = true;
                        return;
                    }
                }
                calibration_step = measure_time_to_fully_open;
            } break;
        }
    }
}
} // namespace SlidingGate

