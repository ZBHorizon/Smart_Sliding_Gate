//////////////// GateSimulator Implementation ////////////////
#include <simulator.hpp>
#include <SlidingGate/Initialize.hpp>
#include <mainwindow.hpp>
#include <test.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono;

namespace SlidingGate {
void GateSimulator::simulation_loop() {
    while (true) {
    for (int i = 0; i < 100; ++i) {
        // Simulate gate movement
        float position = static_cast<float>(i) / 100.0f;
        if(MainWindow::s_instance) MainWindow::s_instance->updateGateProgress(position);
        std::this_thread::sleep_for(100ms);
    }
    //if(MainWindow::s_instance) MainWindow::s_instance->updateGateProgress(0.0f);
    std::this_thread::sleep_for(1000ms);
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
