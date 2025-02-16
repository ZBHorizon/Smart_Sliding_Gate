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

// WiringPi includes (Raspberry Pi)
#include <wiringPi.h>
#include <Initialize.hpp>

using namespace std::chrono;

namespace SlidingGate {

//! Mutex to protect all static motor data.
static std::mutex motor_mutex;

//------------------------------------------------------------------------------
// Now only provide method definitions for Motor, whose declaration is in Motor.hpp
//------------------------------------------------------------------------------

/*!
 * \brief Calculates brake time in ms based on the given speed.
 * \param speed The current speed of the motor.
 * \return The calculated brake time in milliseconds.
 */
milliseconds Motor::calculate_brake_time(int8_t speed) {
    uint8_t steps = std::abs(speed) / Motor::Param::step;
    return steps * Motor::Param::motor_ramp;
}

/*!
 * \brief Opens the gate fully.
 */
void Motor::open() {
    { // lock scope only for updating shared data
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_speed = Motor::Param::max_speed;
    }
    // Additional logic can be added here if needed
}

/*!
 * \brief Closes the gate fully.
 */
void Motor::close() {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_speed = -Motor::Param::max_speed;
    }
    // Additional logic can be added here if needed
}

/*!
 * \brief Stops the motor immediately.
 */
void Motor::stop() {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_speed = 0;
    }
    // Optionally, immediately cut motor power:
    pwmWrite(Pin::PWM, 0);
}

/*!
 * \brief Opens the gate to 50%.
 */
void Motor::half_open() {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_position = 50;
    }
    move_to_position(50);
}

/*!
 * \brief Moves the gate to a specific position (0-100%).
 * \param position The target position of the gate.
 */
void Motor::move_to_position(uint8_t position) {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_position = position;
    }
    move_position_time_based(current_position, desired_position);
}

/*!
 * \brief Moves from current_position to target_position using the time-based offset approach.
 *        This function will block until target_position is reached or until desired_position changes.
 * \param start_pos The starting position of the gate.
 * \param target_pos The target position of the gate.
 */
void Motor::move_position_time_based(std::uint8_t start_pos, std::uint8_t target_pos) {
    if (start_pos == target_pos)
        return;

    milliseconds start_time_ms = (time_to_open * start_pos) / 100;
    milliseconds end_time_ms = (time_to_open * target_pos) / 100;

    bool moving_forward = (target_pos > start_pos);
    {   // Set initial speed inside a short lock scope
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_speed = moving_forward ? Motor::Param::max_speed : -Motor::Param::max_speed;
    }

    auto motion_start = steady_clock::now();
    while (true) {
        { // briefly acquire lock to check for changes
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (desired_position != target_pos)
                break;
        }

        auto now = steady_clock::now();
        auto delta = duration_cast<milliseconds>(now - motion_start).count();
        long long total_ms = static_cast<long long>(time_to_open.count());
        long long cur_offset = static_cast<long long>(start_time_ms.count());
        long long current_time_in_ms = moving_forward ? (cur_offset + delta)
                                                     : (cur_offset - delta);
        if (current_time_in_ms < 0)
            current_time_in_ms = 0;
        if (current_time_in_ms > total_ms)
            current_time_in_ms = total_ms;

        milliseconds brake_time = calculate_brake_time(Motor::Param::max_speed);
        long long target_offset_ms = end_time_ms.count();
        long long distance_to_target = moving_forward
                                       ? (target_offset_ms - current_time_in_ms)
                                       : (current_time_in_ms - target_offset_ms);
        if (distance_to_target < 0)
            distance_to_target = 0;
        if (distance_to_target < brake_time.count()) {
            {
                std::lock_guard<std::mutex> lock(motor_mutex);
                desired_speed = moving_forward ? 10 : -10;
            }
        }

        long long new_pos = (current_time_in_ms * 100LL) / total_ms;
        if (new_pos < 0) new_pos = 0;
        if (new_pos > 100) new_pos = 100;
        { 
            std::lock_guard<std::mutex> lock(motor_mutex);
            current_position = static_cast<std::uint8_t>(new_pos);
        }

        { // Check if target reached using a brief lock
            std::lock_guard<std::mutex> lock(motor_mutex);
            if ((moving_forward && current_position >= target_pos) ||
                (!moving_forward && current_position <= target_pos)) {
                current_position = target_pos;
                break;
            }
        }
        std::this_thread::sleep_for(1ms); // sleep outside lock
    }
    { // ensure clean stop
        std::lock_guard<std::mutex> lock(motor_mutex);
        if (current_position == target_pos)
            desired_speed = 0;
    }
}

/*!
 * \brief Continuously adjusts motor position to match desired position.
 *        This function is meant to run in a dedicated thread.
 */
void Motor::motor_position_loop() {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (current_position == desired_position) {
                // release lock and then sleep
                // (lock_guard goes out of scope)
                ;
            }
        }
        std::this_thread::sleep_for(200ms);

        {   // Lock for critical section of position update
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (desired_position == 0) {
                const std::uint8_t near_threshold = 5;
                if (current_position > near_threshold) {
                    std::uint8_t temp_target = near_threshold;
                    desired_speed = -Motor::Param::max_speed;
                    // release lock while waiting:
                    // (lock_guard ends)
                    move_position_time_based(current_position, temp_target);
                }
            }
        }
        {   // Final slow approach for closed position outside lock
            {
                std::lock_guard<std::mutex> lock(motor_mutex);
                if (desired_position == 0) {
                    desired_speed = -10;
                }
            }
            while (true) {
                if (digitalRead(Pin::CLOSE_SWITCH))
                    break;
                {   // Check intermittently if desired_position changed
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    if (desired_position != 0)
                        break;
                }
                std::this_thread::sleep_for(1ms);
            }
            {
                std::lock_guard<std::mutex> lock(motor_mutex);
                desired_speed = 0;
                current_position = 0;
            }
            continue;
        }
        {   // Similar handling for fully OPEN (100%)
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (desired_position == 100) {
                const std::uint8_t near_threshold = 95;
                if (current_position < near_threshold) {
                    std::uint8_t temp_target = near_threshold;
                    desired_speed = Motor::Param::max_speed;
                    // release lock while moving:
                    // (lock_guard ends)
                    move_position_time_based(current_position, temp_target);
                }
            }
        }
        {
            {
                std::lock_guard<std::mutex> lock(motor_mutex);
                if (desired_position == 100)
                    desired_speed = 10;
            }
            while (true) {
                if (digitalRead(Pin::OPEN_SWITCH))
                    break;
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    if (desired_position != 100)
                        break;
                }
                std::this_thread::sleep_for(1ms);
            }
            {
                std::lock_guard<std::mutex> lock(motor_mutex);
                desired_speed = 0;
                current_position = 100;
            }
            continue;
        }
        // For partial move (1..99%), simply call the time-based approach:
        move_position_time_based(current_position, desired_position);
        std::this_thread::sleep_for(1ms);
    }
}

/*!
 * \brief Continuously adjusts motor speed to match desired speed.
 *        This function is meant to run in a dedicated thread.
 */
void Motor::motor_speed_loop() {
    static steady_clock::time_point overcurrent_start;
    static bool overcurrent_active = false;

    while (true) {
        { // short locking for read conditions
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (desired_speed == 0 && current_speed == 0) {
                // release lock then sleep
                ;
            }
        }
        std::this_thread::sleep_for(200ms);

        { // Check gate limit conditions (lock briefly)
            std::lock_guard<std::mutex> lock(motor_mutex);
            if ((current_speed > 0 && digitalRead(Pin::OPEN_SWITCH)) ||
                (desired_speed < 0 && digitalRead(Pin::CLOSE_SWITCH))) {
                current_speed = 0;
                desired_speed = 0;
                pwmWrite(Pin::PWM, 0);
                continue;
            }
        }
        { // Check light barrier outside lock
            if (current_speed > 0 && digitalRead(Pin::LIGHT_BARRIER)) {
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    current_speed = 0;
                }
                pwmWrite(Pin::PWM, 0);
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    desired_speed = -30;
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(motor_mutex);
            current_current = INA226::read_current();
        }

        auto now = steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (current_current > Motor::Param::CURRENT_THRESHOLD) {
                if (!overcurrent_active) {
                    overcurrent_active = true;
                    overcurrent_start = now;
                } else if (duration_cast<milliseconds>(now - overcurrent_start) >= Motor::Param::overcurrent_duration) {
                    current_speed = 0;
                    pwmWrite(Pin::PWM, 0);
                    if (current_speed > 0)
                        desired_speed = 0;
                    else if (current_speed < 0)
                        desired_speed = 10;
                    overcurrent_active = false;
                }
            } else {
                overcurrent_active = false;
            }
        }

        {
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (current_speed != desired_speed) {
                if (std::abs(current_speed - desired_speed) > Motor::Param::tolerance) {
                    if (std::abs(current_speed) < Motor::Param::direction_threshold) {
                        digitalWrite(Pin::DIRECTION, (desired_speed >= 0 ? LOW : HIGH));
                    }
                    if (current_speed < desired_speed)
                        current_speed += Motor::Param::step;
                    else if (current_speed > desired_speed)
                        current_speed -= Motor::Param::step;
                } else {
                    current_speed = desired_speed;
                }
                pwmWrite(Pin::PWM, std::abs(current_speed));
            }
        }
        std::this_thread::sleep_for(Motor::Param::motor_ramp);
    }
}

/*!
 * \brief Performs motor timing calibration.
 *        Moves gate from closed to open and vice versa, measuring times.
 */
void Motor::calibrate_timing() {
    auto overall_start = steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        is_calibrated = false;
        time_to_open = 0ms;
        time_to_close = 0ms;
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
                { std::lock_guard<std::mutex> lock(motor_mutex); desired_speed = -Motor::Param::calibration_speed; }
                if (digitalRead(Pin::OPEN_SWITCH))
                    calibration_step = check_position;
            } break;
            case measure_time_to_fully_open: {
                { std::lock_guard<std::mutex> lock(motor_mutex); desired_speed = Motor::Param::calibration_speed; }
                auto start = steady_clock::now();
                while (!digitalRead(Pin::OPEN_SWITCH)) {
                    if (steady_clock::now() - overall_start > minutes(1))
                        throw std::runtime_error("Calibration (open) timed out!");
                    std::this_thread::sleep_for(1ms); // sleep outside lock
                }
                auto end = steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    time_to_open = duration_cast<milliseconds>(end - start);
                }
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    if (time_to_close != 0ms) {
                        double factor = static_cast<double>(Motor::Param::calibration_speed) / static_cast<double>(Motor::Param::max_speed);
                        time_to_open = duration_cast<milliseconds>(time_to_open * factor);
                        time_to_close = duration_cast<milliseconds>(time_to_close * factor);
                        is_calibrated = true;
                        return;
                    }
                }
                calibration_step = measure_time_to_fully_close;
            } break;
            case measure_time_to_fully_close: {
                { std::lock_guard<std::mutex> lock(motor_mutex); desired_speed = -Motor::Param::calibration_speed; }
                auto start = steady_clock::now();
                while (!digitalRead(Pin::CLOSE_SWITCH)) {
                    if (steady_clock::now() - overall_start > minutes(1))
                        throw std::runtime_error("Calibration (close) timed out!");
                    std::this_thread::sleep_for(1ms);
                }
                auto end = steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(motor_mutex);
                    time_to_close = duration_cast<milliseconds>(end - start);
                    if (time_to_open != 0ms) {
                        double factor = static_cast<double>(Motor::Param::calibration_speed) / static_cast<double>(Motor::Param::max_speed);
                        time_to_open = duration_cast<milliseconds>(time_to_open * factor);
                        time_to_close = duration_cast<milliseconds>(time_to_close * factor);
                        is_calibrated = true;
                        return;
                    }
                }
                calibration_step = measure_time_to_fully_open;
            } break;
        }
    }
}

} // namespace SlidingGate

