#include <SlidingGate/Control.hpp>
#include <SlidingGate/INA226.hpp>
#include <SlidingGate/IO.hpp>
#include <SlidingGate/Initialize.hpp>
#include <SlidingGate/Motor.hpp>
#include <SlidingGate/job.hpp>

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace SlidingGate;

int main() {
  // Initialize GPIO and INA226 sensor; check for errors.
  if (!Pin::initialize_gpio()) {
    std::cerr << "GPIO initialization failed!" << std::endl;
    return 1;
  }
  /*
    if (!INA226::initialize()) {
        std::cerr << "INA226 initialization failed!" << std::endl;
        return 1;
    }
    */
  // Start the motor loop in a separate thread.
  std::thread motor_thread(&Motor::motor_loop);

  while (true) { Control::control_loop(); }

  motor_thread.join();
  return 0;
}