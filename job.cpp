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
    /**
     * @brief Creates a new job based on the current state and target.
     * @param target_keyframe 
     */
    bool job::create_job(keyframe target_keyframe) {
        keyframe start_keyframe { 
            .speed = Motor::read_speed(),
            .position = Motor::read_position()
        };
        if (std::abs(start_keyframe.speed) < MIN_SPEED) {
            start_keyframe.speed = (target_keyframe.position > start_keyframe.position) ? MIN_SPEED : -MIN_SPEED;;
        }

        delete_job();

        // 1. first keyframe always current position and speed
        keyframes.push_back( start_keyframe );

        float target_direction = (target_keyframe.position > start_keyframe.position) ? 1.0f : -1.0f;
        float deceleration_distance = RAMP_DISTANCE;
        
        // 2. brake phase
        if (std::abs(start_keyframe.speed) > MIN_SPEED) {
            //check if Motor is moving in the same direction as the target position
            if (std::signbit(start_keyframe.speed) == std::signbit(-target_direction)) {
                float brake_position = start_keyframe.position + RAMP_DISTANCE * ((start_keyframe.speed > 0.0f) ? 1.0f : -1.0f);
                keyframes.push_back({target_direction * MIN_SPEED, brake_position });
                target_direction = (target_keyframe.position > keyframes.back().position) ? 1.0f : -1.0f;
                keyframes.push_back({target_direction * -MIN_SPEED, brake_position });
            }
        }
        bool skip = false;

        //acceleration phase
        if (std::abs(keyframes.back().speed) < 1.0f) {
            // Berechne die geplante Beschleunigungsstrecke (je geringer die Startgeschwindigkeit, desto länger)
            // Formel: acceleration_distance = RAMP_DISTANCE × (1 − |start_speed|)
            float accelDistancePlanned = RAMP_DISTANCE * (1.0f - std::abs(keyframes.back().speed));
        
            // Berechne die verbleibende Strecke vom aktuellen Punkt (letzter Keyframe) bis zum Ziel
            // Formel: distance = |target_position − current_position|
            float distanceAvailable = std::abs(target_keyframe.position - keyframes.back().position);
        
            // Standardmäßig soll am Ende die volle Zielgeschwindigkeit erreicht werden.
            float newSpeed = TARGET_MAX_SPEED;
        
            // Überprüfe, ob die geplante Beschleunigungsstrecke plus der Bremsstrecke (RAMP_DISTANCE)
            // die verfügbare Strecke überschreiten.
            if ((accelDistancePlanned + RAMP_DISTANCE) > distanceAvailable) {
                // Nicht genügend Strecke für volle Beschleunigungs- und Bremsphase:
                // Setze zunächst key_position als Referenzpunkt:
                float keyPosition = keyframes.back().position + accelDistancePlanned * target_direction;
                
                // Passe die Beschleunigungsstrecke so an, dass sie die Hälfte der verfügbaren Strecke beträgt:
                accelDistancePlanned = distanceAvailable / 2.0f;
                
                // Berechne die tatsächliche Position des Beschleunigungs-Keyframes:
                float accelPosition = keyframes.back().position + accelDistancePlanned * target_direction;
                
                // Vermeide Division durch Null: Falls keyPosition gleich current_position ist, 
                // setze newSpeed einfach auf die maximale Zielgeschwindigkeit.
                if (keyPosition == keyframes.back().position) {
                    newSpeed = target_direction * TARGET_MAX_SPEED;
                } else {
                    // Lineare Interpolation der Geschwindigkeit:
                    // newSpeed = current_speed + ((target_direction * TARGET_MAX_SPEED) - current_speed)
                    //            × ((accelPosition - current_position) / (keyPosition - current_position))
                    newSpeed = keyframes.back().speed + ((target_direction * TARGET_MAX_SPEED) - keyframes.back().speed) *
                               ((accelPosition - keyframes.back().position) / (keyPosition - keyframes.back().position));
                }
                // Setze das Flag, dass der normale Bremsabschnitt übersprungen wird
                skip = true;
                
                // Füge den berechneten Beschleunigungs-Keyframe hinzu
                keyframes.push_back({ newSpeed, accelPosition });
            } else {
                // Normale Beschleunigungsphase:
                // Berechne die Position des Beschleunigungs-Keyframes:
                float accelPosition = keyframes.back().position + accelDistancePlanned * target_direction;
                // Hier gehen wir davon aus, dass der Motor die maximale Geschwindigkeit erreicht:
                newSpeed = target_direction * TARGET_MAX_SPEED;
                keyframes.push_back({ newSpeed, accelPosition });
            }
        }
        
        // deceleration phase
        if (!skip) {
            keyframes.push_back({ target_direction * 1.0f, target_keyframe.position - RAMP_DISTANCE * ((keyframes.back().speed > 0.0f) ? 1.0f : -1.0f)});
        }
        
        // 5. stop at target position
        keyframes.push_back({ 0.0f, target_keyframe.position });
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

        // If 'position' is current_iter's position, 
        //     return current_iter's speed.
        if (current_iter->position == position) return current_iter->speed;
        

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