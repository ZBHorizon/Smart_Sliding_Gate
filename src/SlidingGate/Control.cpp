#include <SlidingGate/Control.hpp>
#include <SlidingGate/job.hpp>
#include <SlidingGate/Initialize.hpp>
#include <SlidingGate/Motor.hpp>
#include <SlidingGate/INA226.hpp>
#include <SlidingGate/IO.hpp>

#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono;
namespace SlidingGate{

void Control::remote_a_isr(){
    if (job::is_job_active()){
        job::stop_motor();
        return;
    }
    // Wenn im HALF_OPEN-Zustand: unterscheide nach Ursprung:
    switch(gateState) {
        case GateState::HALF_OPEN_FROM_OPEN:
            // War von offen → drücke A: schließt
            gateState = GateState::CLOSED;
            break;
        case GateState::HALF_OPEN_FROM_CLOSED:
            // War von geschlossen → drücke A: öffnet
            gateState = GateState::OPEN;
            break;
        case GateState::CLOSED:
            gateState = GateState::OPEN;
            break;
        case GateState::OPEN:
            gateState = GateState::CLOSED;
            break;
        default:
            // Falls in OPENING oder CLOSING: stoppe und setze auf CLOSED
            job::stop_motor();
            gateState = GateState::CLOSED;
            break;
    }
    float target = (gateState == GateState::OPEN) ? 1.0f : 0.0f;
    job::create_job(target);
}

void Control::remote_b_isr(){
    if (!Motor::is_calibrated())
            return;
        if (job::is_job_active()){
            job::stop_motor();
            return;
        }
        // Falls bereits in einem HALF_OPEN Zustand, wechsle zu CLOSED.
        // Andernfalls: von CLOSED oder OPEN in den entsprechenden HALF_OPEN-Zustand.
        switch(gateState) {
            case GateState::HALF_OPEN_FROM_OPEN:
            case GateState::HALF_OPEN_FROM_CLOSED:
                gateState = GateState::CLOSED;
                break;
            case GateState::CLOSED:
                gateState = GateState::HALF_OPEN_FROM_CLOSED;
                break;
            case GateState::OPEN:
                gateState = GateState::HALF_OPEN_FROM_OPEN;
                break;
            default:
                gateState = GateState::CLOSED;
                break;
        }
        float target = ((gateState == GateState::HALF_OPEN_FROM_OPEN) || (gateState == GateState::HALF_OPEN_FROM_CLOSED)) 
                         ? _Param::_HALF_OPEN : 0.0f;
        job::create_job(target);
}

void Control::remote_d_isr(){
    IO::digitalWrite(Pin::GARDEN_DOOR, HIGH);
    std::this_thread::sleep_for(1000ms);
    IO::digitalWrite(Pin::GARDEN_DOOR, LOW);
}

void Control::user_input(){
    std::cout << "Enter target gate percentage or 's' to stop: ";
    std::string input;
    std::cin >> input;
    if (input == "s") {
        job::stop_motor();
    } else {
        float target_position = std::stof(input);
        if (!Motor::is_calibrated()){
            if (target_position != 0.0f && target_position != 100.0f) {
                std::cout << "Nur 0% oder 100% sind erlaubt, da der Motor nicht kalibriert ist.\n";
                return;
            }
            target_position = (target_position == 100.0f) ? 1.0f : 0.0f;
        }
        job::create_job(target_position);
    }
}

void Control::control_loop() {
    // Starte einen separaten Thread, der konstant die Benutzer-Eingabe abfragt.
    
    while (true) {
            user_input();
            std::this_thread::sleep_for(50ms);
    }
    
}
}