//////////////// GateSimulator Implementation ////////////////
#include <simulator.hpp>
#include <SlidingGate/Initialize.hpp>
#include <test.hpp>
namespace SlidingGate {
void GateSimulator::simulation_loop() {

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
