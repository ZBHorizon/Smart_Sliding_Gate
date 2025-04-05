#include <SlidingGate/Initialize.hpp>
#include <chrono>

using namespace std::chrono;

namespace SlidingGate {
/**
 * @brief The GateSimulator class simulates the physical gate movement, and sesnors.
 *
 *   Pin::PWM:  // Hardware PWM pin.
    Pin::DIRECTION:  // Direction control pin.
    Pin::OPEN_SWITCH:  // Open end switch.
    Pin::CLOSE_SWITCH:  // Close end switch.
    Pin::LIGHT_BARRIER:   //Light barrier sensor.
    Pin::REMOTE_A: // Open remote button.
    Pin::REMOTE_B: // Half-open remote button.
    Pin::REMOTE_C: // Close remote button.
    Pin::REMOTE_D: // Garden door remote button.
    Pin::LAMP: // Gate lamp.
    Pin::GARDEN_DOOR:  // Garden door open.

 * PWM is used to control the speed of the gate motor (0.0f -1.0f = 0% - 100%).,
 * DIRECTION is used to control the direction of the gate motor (0.0f = close, 1.0f = open).
 *
 * The OPEN_SWITCH and CLOSE_SWITCH pins are used to detect the end positions of the gate.
 * The LIGHT_BARRIER pin is used to detect if the gate is obstructed.
 * The REMOTE_A, REMOTE_B, REMOTE_C, and REMOTE_D pins are used to control the gate remotely.
 * The LAMP pin is used to control the gate lamp.
 * The GARDEN_DOOR pin is used to control the garden door.
 * The simulation loop will run in a separate thread and will update the state of the gate and the sensors.
 * The time to fully open or close is defined by TIME_TO_OPEN and TIME_TO_CLOSE.
 * 
 * use following functions to control the gate:
 * Test_IO::read_pin(pin);
 * Test_IO::set_pin(pin, value);
 * 
 * 
 */
class GateSimulator {
public:
    static void simulation_loop();

    static void REMOTE_A_pressed(bool pressed);
    static void REMOTE_B_pressed(bool pressed);
    static void REMOTE_C_pressed(bool pressed);
    static void REMOTE_D_pressed(bool pressed);
    static void LIGHT_BARRIER_interrupted(bool pressed);


private:
    struct _Param {
        static constexpr milliseconds TIME_TO_OPEN = 26000ms; // Time to fully open the gate in seconds
        static constexpr milliseconds TIME_TO_CLOSE = 26000ms; // Time to fully close the gate in seconds
        static constexpr microseconds RANDOM = 116us; // Random time to simulate the delay in the gate movement
    };
    
    static float update_position(float pwm, float direction);
    static void update_switch_states();
    inline static float current_position = 0.0f;
    inline static float current_pwm = 0.0f;
    inline static float current_direction = 0.0f;


};
} // namespace SlidingGate