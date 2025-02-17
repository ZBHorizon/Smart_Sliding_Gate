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

// WiringPi includes (Raspberry Pi)
#include <wiringPi.h>
#include <Initialize.hpp>

using namespace std::chrono;

namespace SlidingGate {

//! Mutex to protect all static motor data.
static std::mutex motor_mutex;
// Neue condition_variable zur Synchronisation der Positionsupdates
static std::condition_variable motor_cv;

//------------------------------------------------------------------------------
// Now only provide method definitions for Motor, whose declaration is in Motor.hpp
//------------------------------------------------------------------------------

void Motor::error_handeling(){
    auto startTime = steady_clock::now();
    constexpr auto timeout = 60s; // Timeout anpassen wenn nötig
    {
        std::unique_lock<std::mutex> lock(motor_mutex);
        error_status = Status::None;
        // Warte, bis sich der Zustand ändert oder das Timeout erreicht ist
        if (!motor_cv.wait_until(lock, startTime + timeout, []{ return Motor::desired_position == Motor::current_position; })) {
            error_status = Status::Timeout;
            return;
        }
        if (error_status == Status::None){
            error_status = Status::Success;
        }
    }
}

/*!
 * \brief Opens the gate fully.
 */
Motor::Status Motor::open() {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_position = 0;
    }
    error_handeling();
    return error_status;
}

/*!
 * \brief Closes the gate fully.
 */
Motor::Status Motor::close() {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_position = 100;
    }
    error_handeling();
    return error_status;
}

/*!
 * \brief Stops the motor immediately.
 */
Motor::Status Motor::stop() {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_speed = 0;
    }
    error_handeling();
    return error_status;
}

/*!
 * \brief Opens the gate to 50%.
 */
Motor::Status Motor::half_open() {
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_position = 50;
    }
    error_handeling();
    return error_status;
}

/*!
 * \brief Moves the gate to the end position (open/close).
 * \param state The state of the motor (open/close).
 * \param pin The pin to read the switch state.
 */
void Motor::move_end_position(MotorState state, int pin) {
    { // Lock for critical section of position update
        std::lock_guard<std::mutex> lock(motor_mutex);
        desired_speed = state == MotorState::Open ? (current_position > Param::near_threshold ? -Param::max_speed : -Param::calibration_speed)
                             : (current_position < 100 - Param::near_threshold ? Param::max_speed : Param::calibration_speed);
    }
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        if (digitalRead(pin)) {
            desired_speed = 0;
            if (state == MotorState::Open) {
                current_position = 0;
                current_position_ms = 0ms;
            } else {
                current_position = 100;
                current_position_ms = time_to_open;
            }
        }
    }
}

/*!
 * \brief Updates the current position of the gate.
 */
void Motor::update_current_position() {
    std::lock_guard<std::mutex> lock(motor_mutex);

    // Calculate how the gate is been moving
    time_point now = steady_clock::now();
    milliseconds elapsed = duration_cast<milliseconds>(now - start_timestamp);

    current_position_ms = elapsed + start_position_ms;

    // Calculate current position in percentage 
    if (motor_state == MotorState::Opening) {
        if (digitalRead(Pin::OPEN_SWITCH)) {
            current_position = 100;
            current_position_ms = time_to_open;
        } else {
        current_position = (current_position_ms.count() * 100) / time_to_open.count();
}
    }     else if (motor_state == MotorState::Closing) {
        if (digitalRead(Pin::CLOSE_SWITCH)) {
            current_position = 0;
            current_position_ms = 0ms;
        } else {
        current_position = 100 - (current_position_ms.count() * 100) / time_to_close.count();

        }
    }
    // Signalisiere alle wartenden Threads, dass sich current_position geändert hat.
    motor_cv.notify_all();
}

/*!
 * \brief Continuously adjusts motor position to match desired position.
 *        This function is meant to run in a dedicated thread.
 */
void Motor::motor_position_loop() {
    while (true) {
        uint8_t local_desired, local_current;
        { // Lese den aktuellen Zustand unter Lock
            std::lock_guard<std::mutex> lock(motor_mutex);
            local_desired = desired_position;
            local_current = current_position;
        }
        if (local_desired == local_current) {
            std::this_thread::sleep_for(200ms);
            continue;
        }

        // Aktualisiere Position
        update_current_position();

        { // Lese den (möglicherweise aktualisierten) desired_position
            std::lock_guard<std::mutex> lock(motor_mutex);
            local_desired = desired_position;
        }
        if (local_desired == 100) {
            move_end_position(MotorState::Open, Pin::OPEN_SWITCH);
            std::this_thread::sleep_for(1ms);
            continue;
        } else if (local_desired == 0) {
            move_end_position(MotorState::Close, Pin::CLOSE_SWITCH);
            std::this_thread::sleep_for(1ms);
            continue;
        }
        bool direction = (local_desired > local_current);
        {   // Setze initialen Geschwindigkeitswert
            std::lock_guard<std::mutex> lock(motor_mutex);
            desired_speed = direction ? Param::max_speed : -Param::max_speed;
        }

        // Calculate the time it takes to brake the motor at current speed
        milliseconds brake_time = (abs(current_speed) / Param::step) * Param::motor_ramp;
        
        // Calculate the target position in milliseconds from procentual position
        milliseconds target_ms = (direction ? time_to_open : time_to_close) * desired_position / 100;
            
        milliseconds time_to_target = target_ms - current_position_ms;
                                            
        if (time_to_target < brake_time) {
            { // Lock for short section of position update
                std::lock_guard<std::mutex> lock(motor_mutex);
                desired_speed = 0;
                if  (target_ms <= current_position_ms) {
                    current_speed = 0;
                    desired_position = current_position;
                }
            }
        }   
        std::this_thread::sleep_for(1ms); // Sleep outside lock
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
        { // Kurzer critical section zum Aktualisieren des Motorzustands
            std::unique_lock<std::mutex> lock(motor_mutex);
            if (desired_speed == 0 && current_speed == 0) {  
                if (motor_state != MotorState::None) {
                    stop_timestamp = steady_clock::now();
                    motor_state = MotorState::None;
                }
            }
            else if (desired_speed < 0 && current_speed < 0 && motor_state != MotorState::Closing) {
                motor_state = MotorState::Closing;
                start_timestamp = steady_clock::now();
                start_position_ms = time_to_close * current_position / 100;
            }
            else if (desired_speed > 0 && current_speed > 0 && motor_state != MotorState::Opening) {
                motor_state = MotorState::Opening;
                start_timestamp = steady_clock::now();
                start_position_ms = time_to_open * current_position / 100;
            }
        }
        
        { // Prüfen ob Motor in Ruhe ist und externen Sleep ausführen (kein manuelles Unlock)
            std::unique_lock<std::mutex> lock(motor_mutex);
            if (desired_speed == 0 && current_speed == 0) {
                // Lock wird automatisch am Blockende freigegeben.
                std::this_thread::sleep_for(200ms);
                continue;
            }
        }
        

        { // Check end switches
            std::lock_guard<std::mutex> lock(motor_mutex);
            if ((current_speed > 0 && digitalRead(Pin::OPEN_SWITCH)) ||
                (desired_speed < 0 && digitalRead(Pin::CLOSE_SWITCH))) {
                current_speed = 0;
                desired_speed = 0;
                pwmWrite(Pin::PWM, 0);
                continue;
            }
        }
        
        { // Check light barrier und weitere Anpassungen
            if (current_speed > 0 && digitalRead(Pin::LIGHT_BARRIER)) {
                error_status = Status::LightBarrier;
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
        
        { //check overcurrent
            std::lock_guard<std::mutex> lock(motor_mutex);
            current_current = INA226::read_current();
        }
        
        {
            std::lock_guard<std::mutex> lock(motor_mutex);
            if (current_current > Motor::Param::current_threshold) {
                time_point now = steady_clock::now();
                if (!overcurrent_active) {
                    overcurrent_active = true;
                    overcurrent_start = now;
                } else if (duration_cast<milliseconds>(now - overcurrent_start) >= Motor::Param::overcurrent_duration) {
                    current_speed = 0;
                    pwmWrite(Pin::PWM, 0);
                    error_status = Status::Overcurrent;
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

        { //ramp up/down speed
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

