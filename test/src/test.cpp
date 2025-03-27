#include <test.hpp>
#include <SlidingGate/IO.hpp>
#include <SlidingGate/Initialize.hpp>

namespace SlidingGate {


    int IO::wiringPiSetup() {
        // std::lock_guard<std::mutex> lock(mtx);
        // wiringpi_setup_called = true;
        // std::cout << "[Sim] wiringPiSetup called." << std::endl;
        return 0;
    }

    void IO::pinMode(int pin, int mode) {
        // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before pinMode.");
        // }
        // pin_mode_map[pin] = mode;
        // std::cout << "[Sim] pinMode: Pin " << pin << " set to mode " << static_cast<int>(mode) << "." << std::endl;
    }

    void IO::pullUpDnControl(int pin, int pud) {
        // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before pullUpDnControl.");
        // }
        // if (pin_mode_map.find(pin) == pin_mode_map.end()) {
        //     throw std::runtime_error("Error: pinMode not set for pin " + std::to_string(pin) + " in pullUpDnControl.");
        // }
        // std::cout << "[Sim] pullUpDnControl: Pin " << pin << " set to pull " << static_cast<int>(pud) << "." << std::endl;
    }

    int IO::digitalRead(int pin) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before digitalRead.");
        // }
        // if (pin_mode_map.find(pin) == pin_mode_map.end()) {
        //     throw std::runtime_error("Error: pinMode not set for pin " + std::to_string(pin) + " in digitalRead.");
        // }
        // // Ensure the pin is configured as an input.
        // if (pin_mode_map[pin] != PinMode::INPUT) {
        //     throw std::runtime_error("Error: digitalRead attempted on pin " + std::to_string(pin) + " not configured as INPUT.");
        // }
        // int state = (pin_state_map.count(pin)) ? pin_state_map[pin] : 1; // Default HIGH due to pull-up.
        // std::cout << "[Sim] digitalRead: Pin " << pin << " reads " << state << "." << std::endl;
        // return state;
        return 0;
    }

    void IO::digitalWrite(int pin, int value) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before digitalWrite.");
        // }
        // if (pin_mode_map.find(pin) == pin_mode_map.end()) {
        //     throw std::runtime_error("Error: pinMode not set for pin " + std::to_string(pin) + " in digitalWrite.");
        // }
        // // Allow writing only if the pin is configured as OUTPUT or PWM_OUTPUT.
        // auto mode = pin_mode_map[pin];
        // if (mode != PinMode::OUTPUT && mode != PinMode::PWM_OUTPUT) {
        //     throw std::runtime_error("Error: digitalWrite attempted on pin " + std::to_string(pin) + " not configured as OUTPUT.");
        // }
        // // For digital outputs such as DIRECTION, LAMP, or GARDEN_DOOR, only allow 0 or 1.
        // if ((pin == Pin::DIRECTION || pin == Pin::LAMP || pin == Pin::GARDEN_DOOR) && (value != 0 && value != 1)) {
        //     throw std::runtime_error("Error: Invalid digital value for pin " + std::to_string(pin) + ". Only 0 or 1 allowed.");
        // }
        // pin_state_map[pin] = value;
        // if (pin == Pin::DIRECTION) {
        //     current_direction = value;
        // }
        // std::cout << "[Sim] digitalWrite: Pin " << pin << " set to " << value << "." << std::endl;
    }

    void IO::pwmWrite(int pin, int value) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before pwmWrite.");
        // }
        // if (pin_mode_map.find(pin) == pin_mode_map.end()) {
        //     throw std::runtime_error("Error: pinMode not set for pin " + std::to_string(pin) + " in pwmWrite.");
        // }
        // if (pin_mode_map[pin] != PinMode::PWM_OUTPUT) {
        //     throw std::runtime_error("Error: pwmWrite attempted on pin " + std::to_string(pin) + " not configured as PWM_OUTPUT.");
        // }
        // if (value > static_cast<int>(pwm_range)) {
        //     throw std::runtime_error("Error: pwmWrite value exceeds the set PWM range.");
        // }
        // // Calculate motor speed; here the direction (0 = LOW, 1 = HIGH) influences the sign.
        // current_motor_speed = (value / static_cast<float>(pwm_range)) * ((current_direction == 0) ? 1.0f : -1.0f);
        // std::cout << "[Sim] pwmWrite: Pin " << pin << " PWM value " << value 
        //           << " set. Motor speed: " << current_motor_speed << std::endl;
    }

    void IO::pwmSetMode(int mode) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before pwmSetMode.");
        // }
        // pwm_mode = mode;
        // std::cout << "[Sim] pwmSetMode: Mode " << mode << " set." << std::endl;
    }

    void IO::pwmSetRange(std::uint32_t range) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before pwmSetRange.");
        // }
        // if (range == 0) {
        //     throw std::runtime_error("Error: PWM range cannot be zero.");
        // }
        // pwm_range = range;
        // std::cout << "[Sim] pwmSetRange: Range " << range << " set." << std::endl;
    }

    void IO::pwmSetClock(int divisor) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before pwmSetClock.");
        // }
        // if (divisor <= 0) {
        //     throw std::runtime_error("Error: PWM clock divisor must be positive.");
        // }
        // pwm_clock = divisor;
        // std::cout << "[Sim] pwmSetClock: Divisor " << divisor << " set." << std::endl;
    }

    int IO::waitForInterrupt(int pin, int mS) {
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before waitForInterrupt.");
        // }
        // std::cout << "[Sim] waitForInterrupt: Waiting on pin " << pin << " for " << mS << " ms." << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(mS));
        return 0;
    }

    int IO::wiringPiISR(int pin, int mode, void (*function)(void)) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiISR.");
        // }
        // isr_map[pin] = function;
        // std::cout << "[Sim] wiringPiISR: ISR for pin " << pin << " with mode " << mode << " registered." << std::endl;
        return 0;
    }

    int IO::wiringPiISRStop(int pin) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiISRStop.");
        // }
        // if (isr_map.erase(pin) == 0) {
        //     std::cerr << "[Sim] wiringPiISRStop: No ISR registered for pin " << pin << "." << std::endl;
        // } else {
        //     std::cout << "[Sim] wiringPiISRStop: ISR for pin " << pin << " stopped." << std::endl;
        // }
        return 0;
    }

    int IO::waitForInterruptClose(int pin) {
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before waitForInterruptClose.");
        // }
        // std::cout << "[Sim] waitForInterruptClose: Waiting for close interrupt on pin " << pin << "." << std::endl;
        return 0;
    }

    void IO::delay(std::uint32_t howLong) {
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before delay.");
        // }
        // std::cout << "[Sim] delay: Waiting " << howLong << " ms." << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(howLong));
    }

    void IO::delayMicroseconds(std::uint32_t howLong) {
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before delayMicroseconds.");
        // }
        // std::cout << "[Sim] delayMicroseconds: Waiting " << howLong << " µs." << std::endl;
        // std::this_thread::sleep_for(std::chrono::microseconds(howLong));
    }

    int IO::wiringPiI2CReadReg16(int fd, int reg) {
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CReadReg16.");
        // }
        // std::cout << "[Sim] wiringPiI2CReadReg16: fd " << fd << ", Register " << reg << " read." << std::endl;
        return 123; // Example value
    }

    int IO::wiringPiI2CWriteReg16(int fd, int reg, int data) {
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CWriteReg16.");
        // }
        // std::cout << "[Sim] wiringPiI2CWriteReg16: fd " << fd << ", Register " << reg << ", Data " << data << " written." << std::endl;
        return 0;
    }

    int IO::wiringPiI2CSetup(const int devId) {
        // if (!wiringpi_setup_called) {
        //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CSetup.");
        // }
        // std::cout << "[Sim] wiringPiI2CSetup: Device ID " << devId << " initialized." << std::endl;
        return 1; // Dummy file descriptor
    }

    void Test_IO::simulate_isr(int pin) {
        // // std::lock_guard<std::mutex> lock(mtx);
        // if (isr_map.find(pin) != isr_map.end()) {
        //     std::cout << "[Sim] simulate_isr: Triggering ISR for pin " << pin << "." << std::endl;
        //     isr_map[pin]();
        // } else {
        //     std::cerr << "[Sim] simulate_isr: No ISR registered for pin " << pin << "." << std::endl;
        // }
    }

    // float Test_IO::get_current_motor_speed(){
    //     // return current_motor_speed;
    // }

    // int Test_IO::get_current_direction(){
    //     return current_direction;
    // }

    //////////////// GateSimulator Implementation ////////////////

    bool GateSimulator::running = false;

    void GateSimulator::simulation_loop() {
        using namespace std::chrono;
        // Update interval (adjust as needed)
        auto interval = milliseconds(100);
        running = true;
        while (running) {
            // Capture the current motor speed.
            float motor_speed = 0.0f;// Test_IO::get_current_motor_speed();
            // Determine delta based on direction.
            float delta = 0.0f;
            if (motor_speed > 0.0f) {
                // Opening: full open takes TIME_TO_OPEN.
                delta = motor_speed * (interval.count() / static_cast<float>(TIME_TO_OPEN.count()));
            } else if (motor_speed < 0.0f) {
                // Closing: full close takes TIME_TO_CLOSE.
                delta = motor_speed * (interval.count() / static_cast<float>(TIME_TO_CLOSE.count()));
            }
            {
                // std::lock_guard<std::mutex> lock(sim_mtx);
                current_gate_position += delta;
                // Clamp the position between 0.0f and 1.0f.
                if (current_gate_position > 1.0f) {
                    current_gate_position = 1.0f;
                } else if (current_gate_position < 0.0f) {
                    current_gate_position = 0.0f;
                }
            }
            // std::cout << "[Sim] Gate Position updated: " << get_gate_position() * 100.0f << "%" << std::endl;
            std::this_thread::sleep_for(interval);
        }
    }

} // namespace SlidingGate
