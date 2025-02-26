/**
 * @file job.cpp
 * @brief Implementation of the job class for motion sequence handling.
 *
 * This file implements methods for creating, deleting, and managing
 * motion jobs. Each keyframe operation is thread-safe.
 */

#include <job.hpp>
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

#include <Motor.hpp>

namespace SlidingGate {
// Definitions for static members
std::list<job::keyframe> job::keyframes;
std::list<job::keyframe>::iterator job::current_iter = job::keyframes.end();
bool job::ready = false;

/**
 * @brief Creates a new job based on the current state and a target keyframe.
 *
 * Calculates intermediate keyframes for acceleration, deceleration, and stopping.
 *
 * @param target_keyframe The desired keyframe.
 * @return true if the job is successfully created.
 */
bool job::create_job(keyframe target_keyframe) {
    //std::lock_guard<std::mutex> lock(job_mutex);

    // Read current speed and position from the motor.
    keyframe start_keyframe { 
        .speed = Motor::read_speed(),
        .position = Motor::read_position()
    };

    // If the starting speed is below the minimum, assign a minimal speed
    // in the direction of the target.
    if (std::abs(start_keyframe.speed) < MIN_SPEED) {
        start_keyframe.speed = (target_keyframe.position > start_keyframe.position) ? MIN_SPEED : -MIN_SPEED;
    }

    // Clear any existing job (keyframes).
    delete_job();

    // Step 1: Add the starting keyframe representing the current state.
    keyframes.push_back(start_keyframe);

    // Determine in which direction the target is.
    float target_direction = (target_keyframe.position > start_keyframe.position) ? 1.0f : -1.0f;
    
    // Step 2: If the current speed is above the minimum,
    // create keyframes for a controlled deceleration phase.
    if (std::abs(start_keyframe.speed) > MIN_SPEED) {
        // Check if the motor is moving in the opposite direction to the target.
        if (std::signbit(start_keyframe.speed) == std::signbit(-target_direction)) {
            // Calculate the braking position.
            float brake_position = start_keyframe.position + RAMP_DISTANCE * ((start_keyframe.speed > 0.0f) ? 1.0f : -1.0f);
            // Add a keyframe that sets the speed to minimum in the direction of the target.
            keyframes.push_back({target_direction * MIN_SPEED, brake_position});
            // Recalculate the target direction based on the new keyframe position.
            target_direction = (target_keyframe.position > keyframes.back().position) ? 1.0f : -1.0f;
            // Add a keyframe to decelerate to negative minimum speed, aiding in smooth transition.
            keyframes.push_back({target_direction * -MIN_SPEED, brake_position});
        }
    }
    bool skip = false;

    // Acceleration phase: If the speed from the last keyframe is less than full speed.
    if (std::abs(keyframes.back().speed) < 1.0f) {
        // Calculate a planned acceleration distance.
        float accelDistancePlanned = RAMP_DISTANCE * (1.0f - std::abs(keyframes.back().speed));
        // Calculate available distance to reach the target.
        float distanceAvailable = std::abs(target_keyframe.position - keyframes.back().position);
        float newSpeed = TARGET_MAX_SPEED;
    
        // If the planned acceleration distance plus ramp distance overshoots the available distance.
        if ((accelDistancePlanned + RAMP_DISTANCE) > distanceAvailable) {
            // Calculate a suitable key position.
            float keyPosition = keyframes.back().position + accelDistancePlanned * target_direction;
            // Adjust the acceleration distance to half of the available distance.
            accelDistancePlanned = distanceAvailable / 2.0f;
            float accelPosition = keyframes.back().position + accelDistancePlanned * target_direction;
            // If the key position did not advance, set speed to maximum.
            if (keyPosition == keyframes.back().position) {
                newSpeed = target_direction * TARGET_MAX_SPEED;
            } else {
                // Interpolate linearly to determine new speed.
                newSpeed = keyframes.back().speed + ((target_direction * TARGET_MAX_SPEED) - keyframes.back().speed) *
                           ((accelPosition - keyframes.back().position) / (keyPosition - keyframes.back().position));
            }
            skip = true;
            keyframes.push_back({newSpeed, accelPosition});
        } else {
            // Otherwise, simply calculate the acceleration keyframe.
            float accelPosition = keyframes.back().position + accelDistancePlanned * target_direction;
            newSpeed = target_direction * TARGET_MAX_SPEED;
            keyframes.push_back({newSpeed, accelPosition});
        }
    }
    
    // Deceleration phase: If the acceleration phase did not already force a skip.
    if (!skip) {
        // Add a keyframe that decelerates before reaching the target.
        keyframes.push_back({target_direction * 1.0f,
                             target_keyframe.position - RAMP_DISTANCE * ((keyframes.back().speed > 0.0f) ? 1.0f : -1.0f)});
    }
    
    // Step 5: Add a final keyframe to fully stop at the target position.
    keyframes.push_back({0.0f, target_keyframe.position});
    current_iter = keyframes.begin();
    ready = true;
    Motor::motor_cv.notify_all();
    return true;
}

/**
 * @brief Deletes all keyframes in the current job.
 */
void job::delete_job() {
    // Lock the mutex and clear all keyframes.
    //std::lock_guard<std::mutex> lock(job_mutex);
    keyframes.clear();
}

/**
 * @brief Stops the motor by generating a stop keyframe.
 */
void job::stop_motor(){
    //std::lock_guard<std::mutex> lock(job_mutex);
    float stop_position;
    // Determine the stop position based on current direction.
    if (Motor::read_speed() > 0.0f) {
        stop_position = Motor::read_position() + RAMP_DISTANCE;
    } else {
        stop_position = Motor::read_position() - RAMP_DISTANCE;
    }
    keyframe stop_keyframe { 
        .speed = 0.0f,
        .position = stop_position,
    };
    create_job(stop_keyframe);
}

/**
 * @brief Checks whether a job is currently active.
 *
 * @return true if there is at least one keyframe.
 */
bool job::is_job_active() {
    // Returns true if there is at least one keyframe.
    //std::lock_guard<std::mutex> lock(job_mutex);
    return !keyframes.empty();
}

/**
 * @brief Returns an interpolated speed value for a given position.
 *
 * If the position falls between two keyframes, the speed is linearly interpolated.
 * If the position is out-of-range, signaling_NaN() is returned.
 *
 * @param position The current position.
 * @return The interpolated speed or signaling_NaN() if invalid.
 */
float job::get_speed(float position) {
    // Lock the mutex to safely access the keyframes.
    //std::lock_guard<std::mutex> lock(job_mutex);
    auto next_iter = std::next(current_iter);

    // If the current iterator's position exactly matches the request.
    if (current_iter->position == position) return current_iter->speed;
    
    // Loop through pairs of keyframes.
    for (; next_iter != keyframes.end(); ++current_iter, ++next_iter) {
        // If the difference in positions is below tolerance, return speed of the next keyframe.
        if (std::abs(next_iter->position - position) < TOLERANCE) return next_iter->speed;
        // If the current segment does not contain the requested position, continue.
        if ((current_iter->position < next_iter->position) == (next_iter->position < position)) continue;
        // If the requested position is out-of-range in the segment, return signaling NaN.
        if ((position < current_iter->position) == (current_iter->position < next_iter->position))
            return std::numeric_limits<float>::signaling_NaN();

        // Interpolate linearly for the requested position.
        float interpolated_speed =
            current_iter->speed +
            ((next_iter->speed - current_iter->speed) *
             (position - current_iter->position) /
             (next_iter->position - current_iter->position));
        return interpolated_speed;
    }
    return std::numeric_limits<float>::signaling_NaN();
}
} // namespace SlidingGate