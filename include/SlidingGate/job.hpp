/**
 * @file job.hpp
 * @brief Declaration of the job class for generating and controlling motion sequences.
 *
 * This class defines a keyframe structure and methods for creating,
 * updating, and deleting motion jobs. All operations on _keyframes are protected
 * by a mutex for thread safety.
 */

#pragma once
#include <list>
#include <limits>
#include <cmath>
#include <mutex> // For thread safety
#include <chrono> // For time-related functions

using namespace std::chrono;

namespace SlidingGate {

class job {
public:
    /**
     * @brief Structure representing a keyframe.
     * 
     * A keyframe defines a point in the motion sequence with a specific speed
     * and position.
     */
    struct keyframe {
        float speed;     ///< Speed value at this keyframe.
        steady_clock::time_point timePoint;      ///< Time point of this keyframe.
    };

    static bool ready;

    /**
     * @brief Creates a new job based on the target keyframe.
     * 
     * This function calculates intermediate _keyframes for acceleration,
     * deceleration, and stopping based on the current motor state.
     * All keyframe operations are protected by a mutex.
     *
     * @param target_position Desired keyframe with target speed and position.
     * @return true if the job was successfully created.
     */
    static bool create_job(float target_position);
    
    /**
     * @brief Generates a job to stop the motor.
     */
    static void stop_motor();

    /**
     * @brief Deletes the current job by clearing all _keyframes.
     */
    static void delete_job();

    /**
     * @brief Indicates whether a job is currently active.
     * 
     * A job is considered active if there is at least one keyframe.
     *
     * @return true if active, otherwise false.
     */
    static bool is_job_active();

    /**
     * @brief Returns an interpolated speed value for a given position.
     * 
     * The function loops over the keyframe list and interpolates the speed
     * between two surrounding _keyframes. If the position is out-of-range,
     * a signaling NaN is returned.
     *
     * @return Interpolated speed or signaling_NaN() if the request is invalid.
     */
    static float get_speed();

private:
    static std::list<keyframe> _keyframes;  ///< List of _keyframes for the current job.

    inline static bool skip_deceleration_phase = false; ///< Flag to skip deceleration phase.

    //function to calculate the position in % based on opening time in ms and current time
    static milliseconds position_to_ms(float position, float direction);

    static keyframe compute_braking_keyframe();
    static keyframe compute_acceleration_keyframe(steady_clock::time_point target_time, float target_direction);
    static keyframe compute_deceleration_keyframe(steady_clock::time_point target_time, float target_direction);

    static void print_keyframes();
    
    /**
     * @brief Mutex to protect _keyframes in a multi-threaded environment.
     */
    inline static std::mutex _job_mutex;     

    static constexpr milliseconds _RAMP_TIME_ms = 2000ms;      ///< Distance used for acceleration/deceleration.
    static constexpr float _MAX_SPEED = 1.0f;    ///< Assumed maximum speed magnitude.
    static constexpr milliseconds _TOLERANCE = 20ms; ///< Tolerance for time comparisons.
    static std::list<keyframe>::iterator _current_iter; ///< Iterator to the current keyframe.
};

} // namespace SlidingGate