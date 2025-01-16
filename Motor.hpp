#pragma once
#include <chrono>
using namespace std::chrono_literals;

namespace SlidingGate {
    struct Ramp
    {
        static std::chrono::milliseconds start_Motor;
        static std::chrono::milliseconds stop_Motor;
    };
    struct MotorPosition {
        static int calibration_speed;
    };
}

