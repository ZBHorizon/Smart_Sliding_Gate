/**
 * @file job.cpp
 * @brief Implementation of the job class for motion sequence handling.
 *
 * This file implements methods for creating, deleting, and managing
 * motion jobs. Each keyframe operation is thread-safe.
 */

#include <SlidingGate/job.hpp>
#include <SlidingGate/Motor.hpp>
#include <SlidingGate/Log.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <cstdint>
#include <cstdlib>    // for std::abs
#include <mutex>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <condition_variable>
#include <list>
#include <algorithm>
#include <atomic>

namespace SlidingGate {
// Definitions for static members
std::list<job::keyframe> job::_keyframes;
std::list<job::keyframe>::iterator job::_current_iter = job::_keyframes.end();
bool job::ready = false;

/**
 * @brief Creates a new job based on the current state and a target keyframe.
 *
 * Calculates intermediate _keyframes for acceleration, deceleration, and stopping.
 *
 * @param target_position The desired keyframe.
 * @return true if the job is successfully created.
 */
bool job::create_job(float target_position) {
    //std::lock_guard<std::mutex> lock(_job_mutex);
    delete_job();
    // Read current speed and position from the motor.
    keyframe start_keyframe { 
        .speed = Motor::read_speed(),
        .timePoint = steady_clock::now()
    };
    // Determine in which direction the target is.
    float target_direction = (target_position > Motor::read_position()) ? 1.0f : -1.0f;

    steady_clock::time_point target_timePoint = start_keyframe.timePoint + position_to_ms(target_position, target_direction);
    // Clear any existing job (_keyframes).
    
    // Log current Position and time and target position and time.
    LOG_INFO() << "\033[35mCurrent Position: " << Motor::read_position() * 100.0f << " %\033[0m";
    LOG_INFO() << "\033[35mTarget Position: " << target_position * 100.0f << " %\033[0m";
    LOG_INFO() << "\033[35mTarget Time: " 
               << std::chrono::duration_cast<std::chrono::milliseconds>(target_timePoint - start_keyframe.timePoint).count() 
               << " ms\033[0m";

    // Step 1: Add the starting keyframe representing the current state.
    _keyframes.push_back(start_keyframe);

   
    // Step 2: If braking is needed.
    if (std::abs(start_keyframe.speed) > 0.0f) {
        // Check if moving opposite to the target.
        if (std::signbit(start_keyframe.speed) == std::signbit(-target_direction)) {
            _keyframes.push_back(compute_braking_keyframe());
        }
    }
 
    // If current speed is below max, add an acceleration keyframe.
    if (std::abs(_keyframes.back().speed) < _MAX_SPEED) {
        _keyframes.push_back(compute_acceleration_keyframe(target_timePoint, target_direction));
    }

    // Deceleration phase: add a keyframe _RAMP_TIME_ms before the target time.
    if (!skip_deceleration_phase) {
        _keyframes.push_back(compute_deceleration_keyframe(target_timePoint, target_direction));
    }
    
    // Final keyframe: fully stop at the target time.
    _keyframes.push_back({0.0f, target_timePoint});
    _current_iter = _keyframes.begin();

    print_keyframes();

    ready = true;
    Motor::motor_cv.notify_all();
    return true;
}

/**
 * @brief Deletes all _keyframes in the current job.
 */
void job::delete_job() {
    // Lock the mutex and clear all _keyframes.
    //std::lock_guard<std::mutex> lock(_job_mutex);
    _keyframes.clear();
}

/**
 * @brief Stops the motor by generating a stop keyframe.
 */
void job::stop_motor(){
    //std::lock_guard<std::mutex> lock(_job_mutex);
    float stop_position;
    // Determine the stop position based on current direction.
    if (Motor::read_speed() > 0.0f) {
        stop_position = Motor::read_position() + 0.02f;
    } else {
        stop_position = Motor::read_position() - 0.02f;
    }
    create_job(stop_position);
}

/**
 * @brief Checks whether a job is currently active.
 *
 * @return true if there is at least one keyframe.
 */
bool job::is_job_active() {
    // Returns true if there is at least one keyframe.
    //std::lock_guard<std::mutex> lock(_job_mutex);
    return !_keyframes.empty();
}

/**
 * @brief Returns an interpolated speed value for a given position.
 *
 * If the position falls between two _keyframes, the speed is linearly interpolated.
 * If the position is out-of-range, signaling_NaN() is returned.
 *
 * @param position The current position.
 * @return The interpolated speed or signaling_NaN() if invalid.
 */
float job::get_speed() {
    steady_clock::time_point current_timePoint = steady_clock::now(); 
    // Lock the mutex to safely access the _keyframes.
    //std::lock_guard<std::mutex> lock(_job_mutex);
    auto next_iter = std::next(_current_iter);

    // If the current iterator's position exactly matches the request.
    auto diff = (next_iter->timePoint >= current_timePoint) 
        ? next_iter->timePoint - current_timePoint 
        : current_timePoint - next_iter->timePoint;
    if (diff < _TOLERANCE) return _current_iter->speed;
    
    // Loop through pairs of _keyframes.
    for (; next_iter != _keyframes.end(); ++_current_iter, ++next_iter) {
        // If the current iterator's position exactly matches the request.
        auto diff = (next_iter->timePoint >= current_timePoint) 
        ? next_iter->timePoint - current_timePoint 
        : current_timePoint - next_iter->timePoint;
        if (diff < _TOLERANCE) return _current_iter->speed;
        // If the current segment does not contain the requested position, continue.
        if ((_current_iter->timePoint < next_iter->timePoint) == (next_iter->timePoint < current_timePoint)) continue;
        // If the requested position is out-of-range in the segment, return signaling NaN.
        if ((current_timePoint < _current_iter->timePoint) == (_current_iter->timePoint < next_iter->timePoint))
            return std::numeric_limits<float>::signaling_NaN();

        // Interpolate linearly for the requested position.
        float interpolated_speed =
            _current_iter->speed +
            ((next_iter->speed - _current_iter->speed) *
             (current_timePoint - _current_iter->timePoint) /
             (next_iter->timePoint - _current_iter->timePoint));
        return interpolated_speed;
    }
    return std::numeric_limits<float>::signaling_NaN();
}

milliseconds job::position_to_ms(float position, float direction){
    // Calculate total duration based on direction.
    if(direction > 0.0f) {
        return milliseconds(static_cast<int>(position * Motor::get_time_to_open().count()));
    } else {
        return milliseconds(static_cast<int>(position * Motor::get_time_to_close().count()));
    }
}

job::keyframe job::compute_braking_keyframe() {
    steady_clock::time_point brake_timepoint = _keyframes.back().timePoint + _RAMP_TIME_ms;
    return {0.0f, brake_timepoint};
}

job::keyframe job::compute_acceleration_keyframe(steady_clock::time_point target_time, float target_direction){
    float newSpeed = target_direction * _MAX_SPEED;
    steady_clock::time_point current_time = _keyframes.back().timePoint;
    // Available time to reach target.
    milliseconds available_time = std::chrono::duration_cast<milliseconds>(target_time - current_time);
    // Use the default ramp time unless the available time is shorter.

    milliseconds accel_duration = std::chrono::duration_cast<milliseconds>(_RAMP_TIME_ms * (_MAX_SPEED - _keyframes.back().speed));
    steady_clock::time_point accel_timepoint = current_time + accel_duration;

    if (accel_duration + _RAMP_TIME_ms > available_time){
        accel_duration = std::chrono::duration_cast<milliseconds>(available_time) / 2;
        steady_clock::time_point key_timepoint = current_time + accel_duration;
        newSpeed = _keyframes.back().speed + ((target_direction * _MAX_SPEED) - _keyframes.back().speed) *
                               ((accel_timepoint- _keyframes.back().timePoint) / (key_timepoint - _keyframes.back().timePoint));
        skip_deceleration_phase = true;
        accel_timepoint = current_time + accel_duration;
    }

    return {newSpeed, accel_timepoint};
}

job::keyframe job::compute_deceleration_keyframe(steady_clock::time_point target_time, float target_direction) {
    steady_clock::time_point decel_timepoint = target_time - _RAMP_TIME_ms;
    return{target_direction * _MAX_SPEED, decel_timepoint};
}

void job::print_keyframes() {
    if (_keyframes.empty()) {
        LOG_INFO() << "\033[35mNo keyframes to display.\033[0m";
        return;
    }

    // Begin with the first keyframe (set as 0ms)
    auto iter = _keyframes.begin();
    int keyframeIndex = 1;
    steady_clock::time_point first_time_point = _keyframes.begin()->timePoint;
    LOG_INFO() << "\033[35mKeyframe: " << keyframeIndex++
               << ", Time: " << 0 << " ms, Speed: " << iter->speed *100.0f << " % \033[0m";
    ++iter;

    // For subsequent keyframes, calculate the time difference (delta) from the previous one.
    for (; iter != _keyframes.end(); ++iter) {
        auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(iter->timePoint - first_time_point).count();
        LOG_INFO() << "\033[35mKeyframe: " << keyframeIndex++
                   << ", Time: " << delta << " ms, Speed: " << iter->speed * 100.0f << " % \033[0m";
    }
}


} // namespace SlidingGate