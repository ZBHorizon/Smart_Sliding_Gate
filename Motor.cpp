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

// WiringPi includes (Raspberry Pi)
#include <wiringPi.h>
#include <Initialize.hpp>
#include <INA226.hpp>
#include <job.hpp>

using namespace std::chrono;

namespace SlidingGate {
//! Mutex to protect all static motor data.
static std::mutex motor_mutex;

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

void Motor::check_end_switches() {
    if ((_motor_state == MotorState::Opening && digitalRead(Pin::OPEN_SWITCH)) ||
        (_motor_state == MotorState::Closing && digitalRead(Pin::CLOSE_SWITCH))) {
        set_speed(0.0f);
        job::delete_job();
    }
}

void Motor::check_light_barrier() {
    // Check if light barrier active when closing
    if (_motor_state == MotorState::Closing && digitalRead(Pin::LIGHT_BARRIER)) { 
        set_speed(0.0f);
        job::keyframe Open { 
            .speed = 0.0f,
            .position = 1.0f
        };
        job::create_job(Open);
    }
    
}

void Motor::check_for_overcurrent(){
    if (INA226::read_current() > Param::current_threshold) {
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
            float position = (_current_position_ms.count() * 100) / _time_to_open.count();
            if (position < 1.0f) {
                _actual_position = position;
            } else {
                _actual_position = 0.95f;
            }
}
    }else if (_motor_state == MotorState::Closing) {
        if (digitalRead(Pin::CLOSE_SWITCH)) {
            _actual_position = 0.0f;
        } else {
            float position = 100.0f - (_current_position_ms.count() * 100.0f) / _time_to_close.count();
            if (position > 0.0f) {
            _actual_position = position;
            } else {
                _actual_position = 0.05f;
            }
        }
    }
}
float Motor::read_position(){
    return _actual_position;
}
void Motor::motor_loop() {
    while (true) {
        //condition variable wait check if job set. muss solange schlafen bis ein neuer job gesetzt wurde wenn kein job mehr anliegt
        std::unique_lock<std::mutex> lock(motor_mutex);
        motor_cv.wait(lock,job::is_job_active());
        check_end_switches();
        check_light_barrier();
        check_for_overcurrent();

        if (update_motor()) {
            job::delete_job();
            continue;
        } 
        // sleep für 1ms (davor muss der mutex unlocken und danach wieder locken)
        lock.unlock();
        std::this_thread::sleep_for(10ms);
    }
}


bool Motor::update_motor() {
    update_current_position();
    float speed = job::get_speed(_actual_position);
    if (std::isnan(speed)) {
        return true;
    }
    set_speed(speed);
    return false;
}

void Motor::set_speed(float speed){
    digitalWrite(Pin::DIRECTION, (speed >= 0 ? LOW : HIGH));
    //set speed
    pwmWrite(Pin::PWM, std::abs(speed));
    _actual_speed = speed;
    update_states();
}

float Motor::read_speed(){
    return _actual_speed;
}

bool Motor::is_calibrated() {
    std::lock_guard<std::mutex> lock(motor_mutex);
    return _is_calibrated;
}

/*!
 * \brief Performs motor timing calibration.
 *        Moves gate from closed to open and vice versa, measuring times.
 */
void Motor::calibrate_timing() {
    auto overall_start = steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        _is_calibrated = false;
        _time_to_open = 0ms;
        _time_to_close = 0ms;
    }
    enum CalibrationStep { move_to_starting_position, check_position, measure_time_to_fully_open, measure_time_to_fully_close };
    CalibrationStep calibration_step = check_position;

    while (true) {
        // Timeout after 1 minute
        if (steady_clock::now() - overall_start > minutes(1))
            throw std::runtime_error("Calibration timed out!");

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
                { std::lock_guard<std::mutex> lock(motor_mutex); set_speed(-Param::calibration_speed); }
                if (digitalRead(Pin::OPEN_SWITCH))
                    calibration_step = check_position;
            } break;
            case measure_time_to_fully_open: {
                { std::lock_guard<std::mutex> lock(motor_mutex); set_speed(Param::calibration_speed); }
                auto start = steady_clock::now();
                while (!digitalRead(Pin::OPEN_SWITCH)) {
                    if (steady_clock::now() - overall_start > minutes(1))
                        throw std::runtime_error("Calibration (open) timed out!");
                    std::this_thread::sleep_for(1ms); // sleep outside lock
                }
                auto end = steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    _time_to_open = duration_cast<milliseconds>(end - start);
                }
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
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
                { std::lock_guard<std::mutex> lock(motor_mutex); set_speed(-Param::calibration_speed); }
                auto start = steady_clock::now();
                while (!digitalRead(Pin::CLOSE_SWITCH)) {
                    if (steady_clock::now() - overall_start > minutes(1))
                        throw std::runtime_error("Calibration (close) timed out!");
                    std::this_thread::sleep_for(1ms);
                }
                auto end = steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
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

