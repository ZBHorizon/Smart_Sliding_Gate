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

milliseconds average(std::list<milliseconds> list){
    milliseconds sum = 0ms;
    sum -= *std::minmax_element(list.begin(), list.end()).first;
    return sum / list.size();
}

std::list<milliseconds> Motor::append_time(std::list<milliseconds> list, milliseconds time){
    if (list.size() >= _Param::LIST_SIZE){
        list.pop_front();
    }
    list.push_back(time);
    return list;
}


/**
 * @brief Updates the internal state of the motor based on the current speed.
 */
void Motor::update_states() {
    if (_actual_speed == 0) {  
        if (_motor_state != MotorState::None) {
            if (_is_time_to_open_calibrated)_stop_timestamp = steady_clock::now();
            _motor_state = MotorState::None;
            std::cout << "Motorstate::None" << std::endl;
        }
    }
    else if (_actual_speed < 0 && _motor_state != MotorState::Closing) {
        _motor_state = MotorState::Closing;
        _start_timestamp = steady_clock::now();
        _start_position_ms = duration_cast<milliseconds>(_time_to_close * _actual_position);
        std::cout << "Motorstate::Closing" << std::endl;
    }
    else if (_actual_speed > 0 && _motor_state != MotorState::Opening) {
        _motor_state = MotorState::Opening;
        _start_timestamp = steady_clock::now();
        _start_position_ms = duration_cast<milliseconds>(_time_to_open * _actual_position);
        std::cout << "Motorstate::Opening" << std::endl;
    }
}

/**
 * @brief Checks if any of the end switches are activated.
 */
bool Motor::check_end_switches() {
    if ((_motor_state == MotorState::Opening && !digitalRead(Pin::OPEN_SWITCH)) ||
        (_motor_state == MotorState::Closing && !digitalRead(Pin::CLOSE_SWITCH)) || 
        (_motor_state == MotorState::Closing && !digitalRead(Pin::LIGHT_BARRIER))) {
        std::cout << "check_end_switches() true" << std::endl;
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
        set_speed(0.0f);
        job::create_job(1.0f);
    }
    
}

/**
 * @brief Checks for an overcurrent condition and takes appropriate action.
 */
void Motor::check_for_overcurrent(){
    if (INA226::readCurrent_mA() > _Param::_CURRENT_THRESHOLD) {
        //if motor is after acceleration phase then stop instantly
        if (std::abs(_actual_speed) >= 0.95f) {
            _overcurrent_active = true;
        }
        //otherwise check if overcurrent is active for a certain duration
        auto now = steady_clock::now();
        if (!_overcurrent_active) {
            _overcurrent_active = true;
            _overcurrent_start = now;
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

/*!
 * \brief Updates the current position of the gate.
 */
void Motor::update_current_position() {
    // Calculate how the gate is been moving
    auto now = steady_clock::now();
    milliseconds current_position_ms = duration_cast<milliseconds>(now - _start_timestamp) + _start_position_ms;

    // Calculate current position in percentage 
    if (_motor_state == MotorState::Opening) {
        if (!digitalRead(Pin::OPEN_SWITCH)) {
            _actual_position = 1.0f;
        } else {
            float position = (static_cast<float>(current_position_ms.count()) * 100.0f) / 
                             static_cast<float>(_time_to_open.count());
            if ((position > 0.99f) && (_slow_speed == 0.0f)){
                _slow_speed = _actual_speed;
                _slow_timestamp = std::chrono::steady_clock::now();
            }
            _actual_position = (position > 0.99f) ? 0.95f : position;
        }
    }else if (_motor_state == MotorState::Closing) {
        if (!digitalRead(Pin::CLOSE_SWITCH)) {
            _actual_position = 0.0f;
        } else {
            float position = 100.0f - (static_cast<float>(current_position_ms.count()) * 100.0f) / 
                             static_cast<float>(_time_to_close.count());
            if ((position < 0.01f) && (_slow_speed == 0.0f)){
                _slow_speed = std::abs(_actual_speed);
                _slow_timestamp = std::chrono::steady_clock::now();
            }
            _actual_position = (position < 0.01f) ? 0.05f : position;
        }
    }
}

void Motor::move_to_starting_position(float starting_position){
    if (starting_position == 1.0f){
        std::cout << "open with calibration speed"
        << std::endl;
        std::unique_lock<std::mutex> lock(motor_mutex);
        set_speed(-_Param::_CALIBRATION_SPEED); 
        _open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
        open_switch_triggered = false; // reset flag after wake-up
        return;
    }
    if (starting_position == 0.0f){
        std::cout << "close with calibration speed"
        << std::endl;
        std::unique_lock<std::mutex> lock(motor_mutex);
        set_speed(_Param::_CALIBRATION_SPEED); 
        _close_switch_cv.wait(lock, [&] { return close_switch_triggered; });
        close_switch_triggered = false; // reset flag after wake-up
        return;
    }
    std::cout << "position needs to be 0.0f or 1.0f"
    << std::endl;

}


void Motor::update_times (){
    static std::mutex update_times_mutex;
    if (digitalRead(Pin::CLOSE_SWITCH)){
        if (!_is_time_to_open_calibrated){
            std::cout << "measure time to open"
                    << std::endl;
            std::unique_lock<std::mutex> lock(update_times_mutex);
            _open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
            open_switch_triggered = false; // reset flag after wake-up
            milliseconds fast_duration = duration_cast<milliseconds>(_slow_timestamp - _start_timestamp);
            milliseconds slow_duration = duration_cast<milliseconds>((_stop_timestamp - _slow_timestamp) / _slow_speed);
            _time_to_open = fast_duration + slow_duration;
            avarage_time_to_open.push_back(_time_to_open);
            _slow_speed = 0.0f;
            _is_time_to_open_calibrated = true;
            return;            
        } 
        std::unique_lock<std::mutex> lock(update_times_mutex);
        _open_switch_cv.wait(lock, [&] { return open_switch_triggered; });
        open_switch_triggered = false; // reset flag after wake-up
        if (digitalRead(Pin::OPEN_SWITCH)){
            milliseconds open_duration = duration_cast<milliseconds>(_stop_timestamp - _start_timestamp);
            avarage_time_to_open = append_time(avarage_time_to_open, open_duration);
            _time_to_open = average(avarage_time_to_open); //return the avrage value of all elemts in list
        }
        return;
    }
    if (digitalRead(Pin::OPEN_SWITCH)){
        if (!_is_time_to_close_calibrated){
            std::cout << "measure time to close"
            << std::endl;
            std::unique_lock<std::mutex> lock(update_times_mutex);
            _close_switch_cv.wait(lock, [&] { return close_switch_triggered; });
            close_switch_triggered = false; // reset flag after wake-up
            milliseconds fast_duration = duration_cast<milliseconds>(_slow_timestamp - _start_timestamp);
            milliseconds slow_duration = duration_cast<milliseconds>((_stop_timestamp - _slow_timestamp) / _slow_speed);
            _time_to_close = fast_duration + slow_duration;
            avarage_time_to_close.push_back(_time_to_close);
            _slow_speed = 0.0f;
            _is_time_to_close_calibrated = true;
            return;         
        } 
        std::unique_lock<std::mutex> lock(update_times_mutex);
        _close_switch_cv.wait(lock, [&] { return close_switch_triggered; });
        close_switch_triggered = false; // reset flag after wake-up
        if (digitalRead(Pin::CLOSE_SWITCH)){
            milliseconds close_duration = duration_cast<milliseconds>(_stop_timestamp - _start_timestamp);
            avarage_time_to_close = append_time(avarage_time_to_close, close_duration);
            _time_to_close = average(avarage_time_to_close); //return the avrage value of all elemts in list
        }
        return;
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
    while (true) {
        //condition variable wait check if job set. muss solange schlafen bis ein neuer job gesetzt wurde wenn kein job mehr anliegt
        std::unique_lock<std::mutex> lock(motor_mutex);
        if (!job::is_job_active()) {
            motor_cv.wait(lock, [&] { return job::ready; });
            std::thread time_thread(&Motor::update_times);
            job::ready = false;
        }

        //check_for_overcurrent();

        if (!update_motor()) {
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
        return false;
    }
    update_states();
    if (check_end_switches()) speed = 0.0f;
    set_speed(speed);
    return true;
}

/**
 * @brief Sets the motor speed and updates its state.
 * @param speed Target speed.
 */
void Motor::set_speed(float speed){
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
    return _is_time_to_open_calibrated && _is_time_to_close_calibrated;
}

// isr for end switches
void Motor::open_switch_isr(){
    std::cout << "open_switch_isr()" << std::endl;
    if ((_motor_state == MotorState::Closing)) return;
    set_speed(0.0f);
    job::delete_job();
    if (!_is_time_to_open_calibrated) return;
    _stop_timestamp = steady_clock::now();
    open_switch_triggered = true;
    _close_switch_cv.notify_all();
};

void Motor::close_switch_isr(){
    std::cout << "close_switch_isr()" << std::endl;
    if (_motor_state == MotorState::Opening)return;
    set_speed(0.0f);
    job::delete_job();
    if (!_is_time_to_close_calibrated) return;
    _stop_timestamp = steady_clock::now();
    close_switch_triggered = true;
    _close_switch_cv.notify_all();
};
 
} // namespace SlidingGate

