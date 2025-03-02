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
        float position;  ///< Position value (e.g., percentage) at this keyframe.
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
     * @param position Current position.
     * @return Interpolated speed or signaling_NaN() if the request is invalid.
     */
    static float get_speed(float position);

private:
    static std::list<keyframe> _keyframes;  ///< List of _keyframes for the current job.

    /**
     * @brief Mutex to protect _keyframes in a multi-threaded environment.
     */
    inline static std::mutex _job_mutex;     

    static constexpr float _TOLERANCE = 0.01f;        ///< Tolerance for position comparisons.
    static constexpr float _RAMP_DISTANCE = 0.2f;      ///< Distance used for acceleration/deceleration.
    static constexpr float _TARGET_MAX_SPEED = 0.15f;    ///< Assumed maximum speed magnitude.
    static constexpr float _MIN_SPEED = 0.05f;          ///< Minimum allowed operating speed.
    static std::list<keyframe>::iterator _current_iter; ///< Iterator to the current keyframe.
};

} // namespace SlidingGate