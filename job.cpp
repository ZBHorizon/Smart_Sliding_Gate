/**
 * @file job.cpp
 * @brief Implementierung der job-Klasse zur Abwicklung von Bewegungsabläufen.
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
    struct job::keyframe {
        float speed;     ///< Geschwindigkeitswert als float
        float position;  ///< Positionswert als float
    };

    /**
     * @brief Creates a new job based on the current state and target.
     * @param start_keyframe 
     * @param target_keyframe 
     */
    bool job::create_job(keyframe target_keyframe) {
        keyframe start_keyframe { 
            .speed = Motor::read_speed(),
            .position = Motor::read_position()
        };

        delete_job();

        // 1. first keyframe always current position and speed
        keyframes.push_back( start_keyframe );

        float target_direction = (target_keyframe.position > start_keyframe.position) ? 1.0f : -1.0f;
        float deceleration_distance = RAMP_DISTANCE;

        // 2. brake phase
        if (start_keyframe.speed != 0.0f) {
            //check if Motor is moving in the same direction as the target position
            if (std::signbit(start_keyframe.speed) == std::signbit(-target_direction)) {
                    float brake_position = start_keyframe.position + RAMP_DISTANCE * ((start_keyframe.speed > 0.0f) ? 1.0f : -1.0f);
                    keyframes.push_back({ MINIMAL_SPEED * target_direction, brake_position });
                    target_direction = (target_keyframe.position > keyframes.back().position) ? 1.0f : -1.0f;
            }
        }
        
        // 3. acceleration phase
        if (std::abs(start_keyframe.speed) < 1.0f) {
            float acceleration_distance = RAMP_DISTANCE * (1.0f - std::abs(start_keyframe.speed));
            // check if acceleration distance and deceleration distance is too long to reach target position with last keyframe position
            float distance = std::abs(target_keyframe.position - keyframes.back().position);
            if ( (acceleration_distance + RAMP_DISTANCE) > distance) {
                acceleration_distance = distance / 2;
                deceleration_distance = distance / 2;
            }
            float accel_position = keyframes.back().position + acceleration_distance * target_direction;
            keyframes.push_back({ target_direction * 1.0f, accel_position });
        } 

        // deceleration phase
        keyframes.push_back({ target_direction * 1.0f, target_keyframe.position - deceleration_distance * ((keyframes.back().speed > 0.0f) ? 1.0f : -1.0f)});

        // 5. stop at target position
        keyframes.push_back({ 0.0f, target_keyframe.position });

        current_iter = keyframes.begin();
        Motor::motor_cv.notify_all();
        return true;
    }
    /**
     * @brief Deletes the current job.
     */
    void job::delete_job() {
        keyframes.clear();
    }

    /**
     * @brief Stops the motor.
     */
    void job::stop_motor(){
        float stop_position;
        if (Motor::read_speed() > 0.0f) {
            stop_position = Motor::read_position() + 0.05f;
        } else {
            stop_position = Motor::read_position() - 0.5f;
        }
        keyframe stop_keyframe { 
            .speed = 0.0f,
            .position = stop_position,
        };
        create_job(stop_keyframe);
    }

    /**
     * @brief Checks if a job is currently active.
     * @return true if a job is active.
     */
    bool job::is_job_active() {
        return !keyframes.empty();
    }
    /**
     * @brief Returns a speed value for the given position by interpolating between keyframes.
     *
     * This function works for any range of positions (e.g., 0.0% to 100.0%) and speed values 
     * (e.g., -100.0% to +100.0%). 
     */
    float job::get_speed(float position) {

        auto next_iter = std::next(current_iter);

        // If 'position' is extremely close to current_iter's position, 
        //     return current_iter's speed.
        if (std::abs(current_iter->position - position) < TOLERANCE) return current_iter->speed;

        // Loop through the list of keyframe pairs (current_iter, next_iter):
        for (; next_iter != keyframes.end(); ++current_iter, ++next_iter) {
        
            // If 'position' is extremely close to next_iter's position, 
            // return next_iter's speed (exact match to the second keyframe in the pair).
            if (std::abs(next_iter->position - position) < TOLERANCE) return next_iter->speed;

            // Check if 'position' is beyond next_iter in the current direction. 
            // If so, keep looping.
            if ((current_iter->position < next_iter->position) == (next_iter->position < position)) continue;

            // Check if 'position' is under segments range
            // if so, return a signaling NaN.
            if ((position < current_iter->position) == (current_iter->position < next_iter->position)) 
                return std::numeric_limits<float>::signaling_NaN();

            //  'position' lies strictly between current_iter->position and iter2->position.
            //   perform a linear interpolation of the speed:
            //
            //         speed = speed1 + ( (speed2 - speed1)
            //                            * (position - position1)
            //                            / (position2 - position1) )
            float interpolated_speed =
                current_iter->speed +
                ( (next_iter->speed - current_iter->speed) *
                (position - current_iter->position) /
                (next_iter->position - current_iter->position) );

            // Return the computed speed.
            return interpolated_speed;
        }
        
        // If exit the loop, it means 'position' was not within or near any of our keyframe ranges.
        // Return signaling_NaN() to indicate an invalid or out-of-range query.
        return std::numeric_limits<float>::signaling_NaN();
    }
} // namespace SlidingGate