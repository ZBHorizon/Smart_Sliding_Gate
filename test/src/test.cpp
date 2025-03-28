#include <test.hpp>
#include <SlidingGate/IO.hpp>
#include <SlidingGate/Initialize.hpp>
#include <algorithm>
#include <string>


namespace SlidingGate {



/* wiringPiSetup() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiSetup() {
    return Test_IO::wiringPiSetup();
}
int Test_IO::wiringPiSetup() {
    wiringpi_setup_called = true;
    std::cout << "[Sim] wiringPiSetup called." << std::endl;
    return 0;
}

/* pinMode() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pinMode(int pin, int mode) {
    Test_IO::pinMode(pin, mode);
}
void Test_IO::pinMode(int pin, int mode) {
    check_wiringPiSetup();
    pin_states[pin] = PinState{0.0f, 0.0f, mode, 0, nullptr, -1}; // Initialize pin state
    check_Pin(pin, "pinMode");
    std::cout << "[Sim] pinMode: Pin " << pin 
              << " (" << getPinName(pin) << ") set to mode " << pinModeToString(mode) << "." << std::endl;
}

/* pullUpDnControl() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pullUpDnControl(int pin, int pud) {
    Test_IO::pullUpDnControl(pin, pud);
}
void Test_IO::pullUpDnControl(int pin, int pud) {
    check_wiringPiSetup();
    check_Pin(pin, "pullUpDnControl");

    pin_states[pin].pullUpDown = pud; // Set pull-up/down mode
    std::cout << "[Sim] pullUpDnControl: Pin " << pin 
              << " (" << getPinName(pin) << ") set to pull " << pullUpToString(pud) << "." << std::endl;
}

/* digitalRead() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::digitalRead(int pin) {
    return Test_IO::digitalRead(pin);
}
int Test_IO::digitalRead(int pin) {
    check_wiringPiSetup();
    check_Pin(pin, "digitalRead");

    int state = (pin_states[pin].current_signal > 0.0f) ? LOW : HIGH; 

    std::cout << "[Sim] digitalRead: Pin " << pin 
              << " (" << getPinName(pin) << ") reads " << digitalValToString(state) << "." << std::endl;
    return state;
}

/* digitalWrite() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::digitalWrite(int pin, int value) {
    Test_IO::digitalWrite(pin, value);
}
void Test_IO::digitalWrite(int pin, int value) {
    check_wiringPiSetup();
    check_Pin(pin, "digitalWrite");

    // only allow 0 or 1.
    if (value != LOW && value != HIGH) {
        std::cerr << "\033[31m" << "[Sim] Error: Invalid digital value for pin " << pin 
                  << " (" << getPinName(pin) << "), Only LOW or HIGH allowed." << std::endl;
    }
    pin_states[pin].previous_signal = pin_states[pin].current_signal; // Update previous signal
    pin_states[pin].current_signal = (value == LOW) ? 0.0f : 1.0f; 
    std::cout << "[Sim] digitalWrite: Pin " << pin 
              << " (" << getPinName(pin) << ") set to " << digitalValToString(value) << "." << std::endl;
}

/* pwmWrite() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmWrite(int pin, int value) {
    Test_IO::pwmWrite(pin, value);
}
void Test_IO::pwmWrite(int pin, int value) {
    check_wiringPiSetup();
    check_Pin(pin, "digitalWrite");

    if (_global_PWM::PWM_MODE == -1) {
        std::cerr << "\033[31m" << "[Sim] Error: PWM mode not set." << std::endl;
    }
    if (_global_PWM::PWM_CLOCK == -1) {
        std::cerr << "\033[31m" << "[Sim] Error: PWM clock not set." << std::endl;
    }
    if (_global_PWM::PWM_RANGE == -1) {
        std::cerr << "\033[31m" << "[Sim] Error: PWM range not set." << std::endl;
    }
    if (value > _global_PWM::PWM_RANGE || value < 0) {
        std::cerr << "\033[31m" << "[Sim] Error: pwmWrite value for pin " << pin 
                  << " (" << getPinName(pin) << "), exceeds the set PWM range of " << _global_PWM::PWM_RANGE << " ." << std::endl;
    }
    float converted_value = static_cast<float>(value / _global_PWM::PWM_RANGE); 
    pin_states[pin].previous_signal = pin_states[pin].current_signal; // Update previous signal
    pin_states[pin].current_signal = converted_value;
    std::cout << "[Sim] pwmWrite: Pin " << pin 
              << " (" << getPinName(pin) << ") PWM value " << value << " set." << std::endl;
}

/* pwmSetMode() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmSetMode(int mode) {
    Test_IO::pwmSetMode(mode);
}
void Test_IO::pwmSetMode(int mode) {
    check_wiringPiSetup();
    _global_PWM::PWM_MODE = mode;
    std::cout << "[Sim] pwmSetMode: Mode " << pwmModeToString(mode) << " set." << std::endl;
}

/* pwmSetRange() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmSetRange(std::uint32_t range) {
    Test_IO::pwmSetRange(range);
}
void Test_IO::pwmSetRange(std::uint32_t range) {
    check_wiringPiSetup();
    if (range == 0) {
        std::cerr << "\033[31m" << "[Sim] Error:  PWM range cannot be zero . " << std::endl;
    }
    _global_PWM::PWM_RANGE = range;
    std::cout << "[Sim] pwmSetRange: Range " << range << " set." << std::endl;
}

/* pwmSetClock() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmSetClock(int divisor) {
    Test_IO::pwmSetClock(divisor);
}
void Test_IO::pwmSetClock(int divisor) {
    check_wiringPiSetup();
    if (divisor <= 0) {
        std::cerr << "\033[31m" << "[Sim] Error: PWM clock divisor must be positive." << std::endl;
    }
    _global_PWM::PWM_CLOCK = divisor;
    std::cout << "[Sim] pwmSetClock: Divisor " << divisor << " set." << std::endl;
}

/* waitForInterrupt() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::waitForInterrupt(int pin, int mS) {
    return Test_IO::waitForInterrupt(pin, mS);
}
int Test_IO::waitForInterrupt(int pin, int mS) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before waitForInterrupt.");
    // }
    // std::cout << "[Sim] waitForInterrupt: Waiting on pin " << pin << " for " << mS << " ms." << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(mS));
    return 0;
}

/* wiringPiISR() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiISR(int pin, int mode, void (*function)(void)) {
    return Test_IO::wiringPiISR(pin, mode, function);
}
int Test_IO::wiringPiISR(int pin, int mode, void (*function)(void)) {
    check_wiringPiSetup();
    pin_states[pin].isr_mode = mode; // Set ISR mode
    pin_states[pin].isr_function = function; // Store the ISR function
    std::cout << "[Sim] wiringPiISR: ISR for pin " << pin << " with mode " << mode << " registered." << std::endl;
    std::cout << "[Sim] wiringPiISR: ISR for pin " << pin 
              << " (" << getPinName(pin) << ") with mode " << isrModeToString(mode) << " registered." << std::endl;
    return 0;
}

/* wiringPiISRStop() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiISRStop(int pin) {
    return Test_IO::wiringPiISRStop(pin);
}
int Test_IO::wiringPiISRStop(int pin) {
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

/* waitForInterruptClose() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::waitForInterruptClose(int pin) {
    return Test_IO::waitForInterruptClose(pin);
}
int Test_IO::waitForInterruptClose(int pin) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before waitForInterruptClose.");
    // }
    // std::cout << "[Sim] waitForInterruptClose: Waiting for close interrupt on pin " << pin << "." << std::endl;
    return 0;
}

/* delay() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::delay(std::uint32_t howLong) {
    Test_IO::delay(howLong);
}
void Test_IO::delay(std::uint32_t howLong) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before delay.");
    // }
    // std::cout << "[Sim] delay: Waiting " << howLong << " ms." << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(howLong));
}

/* delayMicroseconds() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::delayMicroseconds(std::uint32_t howLong) {
    Test_IO::delayMicroseconds(howLong);
}
void Test_IO::delayMicroseconds(std::uint32_t howLong) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before delayMicroseconds.");
    // }
    // std::cout << "[Sim] delayMicroseconds: Waiting " << howLong << " µs." << std::endl;
    // std::this_thread::sleep_for(std::chrono::microseconds(howLong));
}

/* wiringPiI2CReadReg16() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiI2CReadReg16(int fd, int reg) {
    return Test_IO::wiringPiI2CReadReg16(fd, reg);
}
int Test_IO::wiringPiI2CReadReg16(int fd, int reg) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CReadReg16.");
    // }
    // std::cout << "[Sim] wiringPiI2CReadReg16: fd " << fd << ", Register " << reg << " read." << std::endl;
    return 123; // Example value
}

/* wiringPiI2CWriteReg16() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiI2CWriteReg16(int fd, int reg, int data) {
    return Test_IO::wiringPiI2CWriteReg16(fd, reg, data);
}
int Test_IO::wiringPiI2CWriteReg16(int fd, int reg, int data) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CWriteReg16.");
    // }
    // std::cout << "[Sim] wiringPiI2CWriteReg16: fd " << fd << ", Register " << reg << ", Data " << data << " written." << std::endl;
    return 0;
}

/* wiringPiI2CSetup() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiI2CSetup(const int devId) {
    return Test_IO::wiringPiI2CSetup(devId);
}
int Test_IO::wiringPiI2CSetup(const int devId) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CSetup.");
    // }
    // std::cout << "[Sim] wiringPiI2CSetup: Device ID " << devId << " initialized." << std::endl;
    return 1; // Dummy file descriptor
}


void Test_IO::isr_sim_update(int pin) {
    if (pin_states[pin].isr_mode == -1) return; // No ISR registered

    if (pin_states[pin].current_signal == pin_states[pin].previous_signal) {
        return;
    }
    if (pin_states[pin].isr_mode == INT_EDGE_RISING && !(pin_states[pin].current_signal == 1.0f && pin_states[pin].previous_signal == 0.0f)) {
        return;
    }
    if (pin_states[pin].isr_mode == INT_EDGE_FALLING && !(pin_states[pin].current_signal == 1.0f && pin_states[pin].previous_signal == 0.0f)) {
        return;
    }
    if (pin_states[pin].isr_mode == INT_EDGE_BOTH && pin_states[pin].current_signal == pin_states[pin].previous_signal) {
        return;
    }
    std::thread isr_thread(pin_states[pin].isr_function);
    isr_thread.detach(); // Detach the thread to allow it to run independently
    std::cout << "[Sim] ISR triggered on pin " << pin 
              << " (" << getPinName(pin) << ") with mode " << isrModeToString(pin_states[pin].isr_mode) << "." << std::endl;
}



void Test_IO::set_input(int pin, int value){
        pin_states[pin].previous_signal = pin_states[pin].current_signal; // Update previous signal
        pin_states[pin].current_signal = value;
        isr_sim_update(pin);   
}

// float Test_Test_IO::get_current_motor_speed(){
//     // return current_motor_speed;
// }

// int Test_Test_IO::get_current_direction(){
//     return current_direction;
// }

void Test_IO::check_wiringPiSetup(){
    if (!wiringpi_setup_called) {
        std::cerr << "\033[31m" << "[Sim] Error: wiringPiSetup must be called before." << std::endl;
    }
}

void Test_IO::check_Pin(int pin, std::string function_name) {
    if (std::find(addressable_gpio_pins.begin(), addressable_gpio_pins.end(), pin) == addressable_gpio_pins.end()) {
        std::cerr << "\033[31m" << "[Sim] Error: Pin " << pin << " is not a valid GPIO pin " << " in " << function_name << " ." << std::endl;
    }
    if (pin_states.find(pin) == pin_states.end()) {
        std::cerr << "\033[31m" << "[Sim] Error: pinMode not set for pin " << pin << " in " << function_name << " ." << std::endl;
    }
    if (pin_states[pin].pin_mode != INPUT && function_name == "PullUpDnControl") {
        std::cerr << "\033[31m" << "[Sim] Error: " << function_name << " attempted on pin " << pin << " not configured as INPUT." << std::endl;
    }
    if (pin_states[pin].pin_mode != OUTPUT && function_name == "digitalWrite") {
        std::cerr << "\033[31m" << "[Sim] Error: " << function_name << " attempted on pin " << pin << " not configured as OUTPUT." << std::endl;
    }
    if (pin_states[pin].pin_mode != PWM_OUTPUT && function_name == "pwmWrite") {
        std::cerr << "\033[31m" << "[Sim] Error: " << function_name << " attempted on pin " << pin << " not configured as PWM_OUTPUT." << std::endl;
    }
}

std::string Test_IO::pinModeToString(int mode) {
    switch(mode) {
        case INPUT: return "INPUT";
        case OUTPUT: return "OUTPUT";
        case PWM_OUTPUT: return "PWM_OUTPUT";
        case PWM_MS_OUTPUT: return "PWM_MS_OUTPUT";
        case PWM_BAL_OUTPUT: return "PWM_BAL_OUTPUT";
        case SOFT_PWM_OUTPUT: return "SOFT_PWM_OUTPUT";
        case SOFT_TONE_OUTPUT: return "SOFT_TONE_OUTPUT";
        case PWM_TONE_OUTPUT: return "PWM_TONE_OUTPUT";
        case PM_OFF: return "PM_OFF";
        default: return std::to_string(mode);
    }
}

std::string Test_IO::pullUpToString(int pud) {
    switch(pud) {
        case PUD_OFF: return "PUD_OFF";
        case PUD_DOWN: return "PUD_DOWN";
        case PUD_UP: return "PUD_UP";
        default: return std::to_string(pud);
    }
}

std::string Test_IO::digitalValToString(int value) {
    switch(value) {
        case LOW: return "LOW";
        case HIGH: return "HIGH";
        default: return std::to_string(value);
    }
}

std::string Test_IO::pwmModeToString(int mode) {
    switch(mode) {
        case PWM_MODE_MS: return "PWM_MODE_MS";
        case PWM_MODE_BAL: return "PWM_MODE_BAL";
        default: return std::to_string(mode);
    }
}

std::string Test_IO::isrModeToString(int mode) {
    switch(mode) {
        case INT_EDGE_SETUP: return "INT_EDGE_SETUP";
        case INT_EDGE_FALLING: return "INT_EDGE_FALLING";
        case INT_EDGE_RISING: return "INT_EDGE_RISING";
        case INT_EDGE_BOTH: return "INT_EDGE_BOTH";
        default: return std::to_string(mode);
    }
}


std::string Test_IO::getPinName(int pin) {
    switch(pin) {
        case Pin::PWM:  return "PWM";          // Hardware PWM pin.
        case Pin::DIRECTION:  return "DIRECTION";    // Direction control pin.
        case Pin::OPEN_SWITCH:  return "OPEN_SWITCH";  // Open end switch.
        case Pin::CLOSE_SWITCH:  return "CLOSE_SWITCH"; // Close end switch.
        case Pin::LIGHT_BARRIER:  return "LIGHT_BARRIER"; // Light barrier sensor.
        case Pin::REMOTE_A: return "REMOTE_A";     // Open remote button.
        case Pin::REMOTE_B: return "REMOTE_B";     // Half-open remote button.
        case Pin::REMOTE_C: return "REMOTE_C";     // Close remote button.
        case Pin::REMOTE_D: return "REMOTE_D";     // Garden door remote button.
        case Pin::LAMP: return "LAMP";         // Gate lamp.
        case Pin::GARDEN_DOOR:  return "GARDEN_DOOR";  // Garden door control.
        default: return "";
    }
}

} // namespace SlidingGate
