#pragma once
#include <cstdint>
#include <chrono>

namespace SlidingGate {

    class Motor {
    public:
        // Basic control methods
        static void open();          // Opens the gate fully
        static void close();         // Closes the gate fully
        static void stop();          // Stops the motor immediately
        static void half_open();     // Opens the gate to 50%
        static void move_to_position(uint8_t position); // Moves the gate to a specific position (0-100%)

        // Loop methods for threads
        static void motor_speed_loop(); // Continuously adjusts motor speed to match desired speed
        static void motor_position_loop(); // Continuously adjusts motor position to match desired position

        // Calibration (if needed)
        static void calibrate_timing(); // Calibrates the timing for opening and closing the gate

        // Other helper methods declared in implementation...
    private:
        // Internal static state variables
        
        // Desired speed of the motor (set by control methods)
        inline static int8_t desired_speed = 0;
        // Current speed of the motor (updated in motor_speed_loop)
        inline static int8_t current_speed = 0;
        // Desired position of the gate (percentage, 0-100)
        inline static std::uint8_t desired_position = 0;
        // Current position of the gate (percentage, 0-100)
        inline static std::uint8_t current_position = 0;
        // Time required to fully open the gate (milliseconds) during calibration
        inline static std::chrono::milliseconds time_to_open = 0ms;
        // Time required to fully close the gate (milliseconds) during calibration
        inline static std::chrono::milliseconds time_to_close = 0ms;
        // Flag indicating whether calibration is complete
        inline static bool is_calibrated = false;
        // Current measured by the INA226 sensor in mA
        inline static uint16_t current_current = 0;

        // Declarations for helper methods used in Motor.cpp
        static std::chrono::milliseconds calculate_brake_time(int8_t speed); // Calculates the brake time based on speed
        static void move_position_time_based(std::uint8_t start_pos, std::uint8_t target_pos); // Moves the gate based on time

        // Parameters for timing and speed control; adjust as needed.
        struct Param {
            // Time delay per speed step 
            inline static std::chrono::milliseconds motor_ramp = 1ms;
            // Speed used in calibration (positive = forward, negative = backward)
            inline static uint8_t calibration_speed = 30;
            // Maximum motor speed
            inline static uint8_t max_speed = 100;
            // Speed threshold for range of direction change
            inline static uint8_t direction_threshold = 2;
            // Step size for speed adjustments
            inline static uint8_t step = 1;
            // Tolerance for speed adjustments
            inline static uint8_t tolerance = 3;
            // Current threshold for overcurrent detection (in mA)
            static constexpr uint16_t CURRENT_THRESHOLD = 5000; // 5A in mA
            // Duration for overcurrent detection (milliseconds)
            inline static std::chrono::milliseconds overcurrent_duration = 500ms;
        };
    };

} // namespace SlidingGate
