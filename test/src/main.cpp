#include <SlidingGate/Motor.hpp>
#include <SlidingGate/Initialize.hpp>
#include <SlidingGate/INA226.hpp>
#include <SlidingGate/job.hpp>
#include <SlidingGate/Control.hpp>

#include <SlidingGate/Log/Stream.hpp>

#include <test.hpp>


#include <iostream>
#include <thread>
#include <chrono>

#include <mainwindow.hpp>
#include <QApplication>

using namespace std::chrono;
using namespace SlidingGate;


int main(int argc, char *argv[]){

    

    auto stream = std::make_shared<Log::Stream>(&std::cout, Log::Level::ALL);
    stream->SetColorized(true);
    Log::AddHandler(stream);

    // Initialize GPIO and INA226 sensor; check for errors.
    if (!Pin::initialize_gpio()) {
        LOG_ERROR() << "GPIO initialization failed!";
        return 1;
    } else {
        LOG_INFO() << "GPIO initialization successful!";
    }
    /*
    if (!INA226::initialize()) {
        std::cerr << "INA226 initialization failed!" << std::endl;
        return 1;
    }
    */
    // Start the motor loop in a separate thread.
    std::thread motor_thread(&Motor::motor_loop);

    //std::thread gate_thread(&GateSimulator::simulation_loop);
    std::thread control_thread(&Control::control_loop);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}