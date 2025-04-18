#include <SlidingGate/Control.hpp>
#include <SlidingGate/INA226.hpp>
#include <SlidingGate/Initialize.hpp>
#include <SlidingGate/Log/Stream.hpp>
#include <SlidingGate/Motor.hpp>
#include <SlidingGate/job.hpp>

#include <mainwindow.hpp>
#include <simulator.hpp>
#include <test.hpp>

#include <QApplication>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace SlidingGate;


int main(int argc, char* argv[]) {
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

  std::thread gate_thread(&GateSimulator::simulation_loop);

  std::thread control_thread(&Control::control_loop);

  QApplication a(argc, argv);
  MainWindow   w;
  w.show();

  int ret = a.exec(); // Execute the application event loop
  // Wait for threads to finish before exiting the program.
  motor_thread.join();
  gate_thread.join();
  control_thread.join();
  return ret;
}