//////////////// GateSimulator Implementation ////////////////
#include <simulator.hpp>
#include <SlidingGate/Initialize.hpp>
#include <mainwindow.hpp>
#include <test.hpp>
#include <SlidingGate/Log.hpp>

#include <chrono>
#include <thread>
#include <random>

using namespace std::chrono_literals;

namespace SlidingGate {
void GateSimulator::simulation_loop() {
    // Initialize the gate as closed.
    current_position = 0.0f;
    update_switch_states(current_position);
    // Main simulation loop.
    while (true) {
        // Read PWM and direction values from the hardware interface.
        current_pwm = Test_IO::read_pin(Pin::PWM);
        current_direction = Test_IO::read_pin(Pin::DIRECTION);

        // Validate PWM value range [0.0f, 1.0f].
        if (current_pwm > 1.0f) {
            LOG_ERROR() << "Error: PWM value exceeds 100% (1.0f).";
        } else if (current_pwm < 0.0f) {
            LOG_ERROR() << "Error: PWM value is less than 0% (0.0f).";
        }

        // Update gate position only if PWM is active (greater than 0).
        if (current_pwm > 0.0f) {
            // Calculate new position based on PWM and direction.
            current_position = update_position(current_pwm, current_direction);

            // Update the open/close switch states based on the current position.
            update_switch_states(current_position);
        }
        // Update the gate progress in the main window, if it exists.
        if (MainWindow::s_instance) {
            MainWindow::s_instance->updateGateProgress(current_position);
        }
        // Sleep for 1 millisecond to limit CPU usage.
        std::this_thread::sleep_for(20ms);
    }
}

float GateSimulator::update_position(float pwm, float direction) {
    using clock = std::chrono::steady_clock;
    static auto last_time = clock::now();
    auto current_time = clock::now();
    std::chrono::duration<float> elapsed = current_time - last_time;
    last_time = current_time;

    // Convert time constants from milliseconds to seconds.
    float time_constant = (direction == 1.0f)
        ? (_Param::TIME_TO_OPEN.count() / 1000.0f)
        : (_Param::TIME_TO_CLOSE.count() / 1000.0f);

    // Calculate delta position.
    float delta = pwm * (elapsed.count() / time_constant);

    // Update position based on direction.
    float new_position = (direction == 0.0f) ? (current_position + delta) : (current_position - delta);

    return new_position;
}

void GateSimulator::update_switch_states(float pos) {
    // Update OPEN_SWITCH:
    // If the gate is fully open (position == 1.0) and the open switch is off, then turn it on.
    if (pos >= 1.0f && Test_IO::read_pin(Pin::OPEN_SWITCH) == 0.0f) {
        Test_IO::set_pin(Pin::OPEN_SWITCH, 0.0f);
    }
    // If the gate is not fully open and the open switch is on, then turn it off.
    else if (pos < 1.0f && Test_IO::read_pin(Pin::OPEN_SWITCH) == 1.0f) {
        Test_IO::set_pin(Pin::OPEN_SWITCH, 1.0f);
    }

    // Update CLOSE_SWITCH:
    // If the gate is fully closed (position == 0.0) and the close switch is off, then turn it on.
    if (pos <= 0.0f && Test_IO::read_pin(Pin::CLOSE_SWITCH) == 0.0f) {
        Test_IO::set_pin(Pin::CLOSE_SWITCH, 0.0f);
    }
    // If the gate is not fully closed and the close switch is on, then turn it off.
    else if (pos > 0.0f && Test_IO::read_pin(Pin::CLOSE_SWITCH) == 1.0f) {
        Test_IO::set_pin(Pin::CLOSE_SWITCH, 1.0f);
    }
}

void GateSimulator::REMOTE_A_pressed(bool pressed){
    Test_IO::set_pin(Pin::REMOTE_A, pressed ? 1.0f : 0.0f);
}

void GateSimulator::REMOTE_B_pressed(bool pressed){
    Test_IO::set_pin(Pin::REMOTE_B, pressed ? 1.0f : 0.0f);
}

void GateSimulator::REMOTE_C_pressed(bool pressed){
    Test_IO::set_pin(Pin::REMOTE_C, pressed ? 1.0f : 0.0f);
}

void GateSimulator::REMOTE_D_pressed(bool pressed){
    Test_IO::set_pin(Pin::REMOTE_D, pressed ? 1.0f : 0.0f);
}

void GateSimulator::LIGHT_BARRIER_interrupted(bool pressed){
    Test_IO::set_pin(Pin::LIGHT_BARRIER, pressed ? 1.0f : 0.0f);
}

}
