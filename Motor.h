#pragma once
#include <chrono>
using namespace std::chrono_literals;

namespace Ramp {
	static std::chrono::milliseconds start_Motor = 5ms;
	static std::chrono::milliseconds stop_Motor = 4ms;
}

namespace motor_position {
	int calibration_speed = 100;
}