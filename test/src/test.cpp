#define WIN32_LEAN_AND_MEAN 
#include <Windows.h>
#include <DbgHelp.h>
#ifdef _WIN32
#pragma comment(lib, "Dbghelp.lib")
#endif
#undef INPUT  // remove Windows' INPUT macro

#include <test.hpp>
#include <SlidingGate/Log.hpp>

#include <SlidingGate/IO.hpp>
#include <SlidingGate/Initialize.hpp>

#include <mainwindow.hpp>

#include <algorithm>
#include <string>
#include <cstdlib>

namespace SlidingGate {

/* wiringPiSetup() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiSetup() {
    Test_IO::wiringpi_setup_called = true;
    LOG_INFO() << "wiringPiSetup called." ;
    return 0;
}

/* pinMode() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pinMode(int pin, int mode) {
    Test_IO::check_wiringPiSetup();
    Test_IO::pin_states[pin] = Test_IO::PinState{0.0f, 0.0f, mode, 0, nullptr, -1}; // Initialize pin state
    Test_IO::check_Pin(pin, "pinMode");
    LOG_INFO() << "pinMode: Pin " << Test_IO::ORANGE << pin << Test_IO::RESET
             << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
             << ") set to mode " << Test_IO::ORANGE << Test_IO::pinModeToString(mode) << Test_IO::RESET << ".";
}

/* pullUpDnControl() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pullUpDnControl(int pin, int pud) {
    Test_IO::check_wiringPiSetup();
    Test_IO::check_Pin(pin, "pullUpDnControl");

    // Set pull-up/down mode
    LOG_INFO() << "pullUpDnControl: Pin " << Test_IO::ORANGE << pin << Test_IO::RESET
             << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
             << ") set to pull " << Test_IO::ORANGE << Test_IO::pullUpToString(pud) << Test_IO::RESET << ".";
}

/* digitalRead() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::digitalRead(int pin) {
    Test_IO::check_wiringPiSetup();
    Test_IO::check_Pin(pin, "digitalRead");

    int state = (Test_IO::pin_states[pin].current_signal > 0.0f) ? LOW : HIGH; 

    LOG_INFO() << "digitalRead: Pin " << Test_IO::ORANGE << pin << Test_IO::RESET
             << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
             << ") reads " << Test_IO::ORANGE << Test_IO::digitalValToString(state) << Test_IO::RESET << ".";
    return state;
}

/* digitalWrite() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::digitalWrite(int pin, int value) {
    Test_IO::check_wiringPiSetup();
    Test_IO::check_Pin(pin, "digitalWrite");

    // only allow 0 or 1.
    if (value != LOW && value != HIGH) {
        LOG_ERROR() << "Error: Invalid digital value for pin " << Test_IO::ORANGE << pin << Test_IO::RESET
                  << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
                  << "), only LOW or HIGH allowed.";
    }
    Test_IO::pin_states[pin].previous_signal = Test_IO::pin_states[pin].current_signal; // Update previous signal
    Test_IO::pin_states[pin].current_signal = (value == LOW) ? 0.0f : 1.0f;  
    LOG_INFO() << "digitalWrite: Pin " << Test_IO::ORANGE << pin << Test_IO::RESET
             << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
             << ") set to " << Test_IO::ORANGE << Test_IO::digitalValToString(value) << Test_IO::RESET << ".";
    if(MainWindow::s_instance)
        MainWindow::s_instance->updateTable(pin);
}

/* pwmWrite() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmWrite(int pin, int value) {
    Test_IO::check_wiringPiSetup();
    Test_IO::check_Pin(pin, "pwmWrite");

    if (Test_IO::global_PWM::PWM_MODE == -1) {
        LOG_ERROR() << "Error: PWM mode not set.";
    }
    if (Test_IO::global_PWM::PWM_CLOCK == -1) {
        LOG_ERROR() << "Error: PWM clock not set.";
    }
    if (Test_IO::global_PWM::PWM_RANGE == -1) {
        LOG_ERROR() << "Error: PWM range not set.";
    }
    if (value > Test_IO::global_PWM::PWM_RANGE || value < 0) {
        LOG_ERROR() << "Error: pwmWrite value for pin " << Test_IO::ORANGE << pin << Test_IO::RESET
                  << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
                  << ") exceeds the set PWM range of " << Test_IO::ORANGE << Test_IO::global_PWM::PWM_RANGE << Test_IO::RESET << ".";
    }
    float converted_value = static_cast<float>(value / Test_IO::global_PWM::PWM_RANGE); 
    Test_IO::pin_states[pin].previous_signal = Test_IO::pin_states[pin].current_signal; // Update previous signal
    Test_IO::pin_states[pin].current_signal = converted_value;
    LOG_INFO() << "pwmWrite: Pin " << Test_IO::ORANGE << pin << Test_IO::RESET
             << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
             << ") PWM value " << Test_IO::ORANGE << value << Test_IO::RESET << " set.";
    if(MainWindow::s_instance)
        MainWindow::s_instance->updateTable(pin);
}

/* pwmSetMode() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmSetMode(int mode) {
    Test_IO::check_wiringPiSetup();
    Test_IO::global_PWM::PWM_MODE = mode;
    LOG_INFO() << "pwmSetMode: Mode " << Test_IO::ORANGE << Test_IO::pwmModeToString(mode) << Test_IO::RESET << " set.";
}

/* pwmSetRange() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmSetRange(std::uint32_t range) {
    Test_IO::check_wiringPiSetup();
    if (range == 0) {
        LOG_ERROR() << "Error: PWM range cannot be zero.";
    }
    Test_IO::global_PWM::PWM_RANGE = range;
    LOG_INFO() << "pwmSetRange: Range " << Test_IO::ORANGE << range << Test_IO::RESET << " set.";
}

/* pwmSetClock() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::pwmSetClock(int divisor) {
    Test_IO::check_wiringPiSetup();
    if (divisor <= 0) {
        LOG_ERROR() << "Error: PWM clock divisor must be positive.";
    }
    Test_IO::global_PWM::PWM_CLOCK = divisor;
    LOG_INFO() << "pwmSetClock: Divisor " << Test_IO::ORANGE << divisor << Test_IO::RESET << " set.";
}

/* waitForInterrupt() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::waitForInterrupt(int pin, int mS) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before waitForInterrupt.");
    // }
    // LOG_INFO() << "waitForInterrupt: Waiting on pin " << pin << " for " << mS << " ms." ;
    // std::this_thread::sleep_for(std::chrono::milliseconds(mS));
    return 0;
}

/* wiringPiISR() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiISR(int pin, int mode, void (*function)(void)) {
    Test_IO::check_wiringPiSetup();
    Test_IO::pin_states[pin].isr_mode = mode; // Set ISR mode
    Test_IO::pin_states[pin].isr_function = function; // Store the ISR function
    LOG_INFO() << "wiringPiISR: ISR for pin " << Test_IO::ORANGE << pin << Test_IO::RESET
             << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
             << ") with mode " << Test_IO::ORANGE << Test_IO::isrModeToString(mode) << Test_IO::RESET
             << " and function " << Test_IO::ORANGE << Test_IO::isrfuntionNameToString(function) << Test_IO::RESET << " registered.";
    return 0;
}

/* wiringPiISRStop() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiISRStop(int pin) {
    // // std::lock_guard<std::mutex> lock(mtx);
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiISRStop.");
    // }
    // if (isr_map.erase(pin) == 0) {
    //     LOG_ERROR() << "wiringPiISRStop: No ISR registered for pin " << pin << "." ;
    // } else {
    //     LOG_INFO() << "wiringPiISRStop: ISR for pin " << pin << " stopped." ;
    // }
    return 0;
}

/* waitForInterruptClose() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::waitForInterruptClose(int pin) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before waitForInterruptClose.");
    // }
    // LOG_INFO() << "waitForInterruptClose: Waiting for close interrupt on pin " << pin << "." ;
    return 0;
}

/* delay() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::delay(std::uint32_t howLong) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before delay.");
    // }
    // LOG_INFO() << "delay: Waiting " << howLong << " ms." ;
    // std::this_thread::sleep_for(std::chrono::milliseconds(howLong));
}

/* delayMicroseconds() */
/*------------------------------------------------------------------------------------------------------------------*/
void IO::delayMicroseconds(std::uint32_t howLong) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before delayMicroseconds.");
    // }
    // LOG_INFO() << "delayMicroseconds: Waiting " << howLong << " µs." ;
    // std::this_thread::sleep_for(std::chrono::microseconds(howLong));
}

/* wiringPiI2CReadReg16() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiI2CReadReg16(int fd, int reg) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CReadReg16.");
    // }
    // LOG_INFO() << "wiringPiI2CReadReg16: fd " << fd << ", Register " << reg << " read." ;
    return 123; // Example value
}

/* wiringPiI2CWriteReg16() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiI2CWriteReg16(int fd, int reg, int data) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CWriteReg16.");
    // }
    // LOG_INFO() << "wiringPiI2CWriteReg16: fd " << fd << ", Register " << reg << ", Data " << data << " written." ;
    return 0;
}

/* wiringPiI2CSetup() */
/*------------------------------------------------------------------------------------------------------------------*/
int IO::wiringPiI2CSetup(const int devId) {
    // if (!wiringpi_setup_called) {
    //     throw std::runtime_error("Error: wiringPiSetup must be called before wiringPiI2CSetup.");
    // }
    // LOG_INFO() << "wiringPiI2CSetup: Device ID " << devId << " initialized." ;
    return 1; // Dummy file descriptor
}

void Test_IO::isr_sim_update(int pin) {
    if (Test_IO::pin_states[pin].isr_mode == -1) return; // No ISR registered

    if (Test_IO::pin_states[pin].current_signal == Test_IO::pin_states[pin].previous_signal) {
        return;
    }
    if (Test_IO::pin_states[pin].isr_mode == INT_EDGE_RISING && !(Test_IO::pin_states[pin].current_signal == 1.0f && Test_IO::pin_states[pin].previous_signal == 0.0f)) {
        return;
    }
    if (Test_IO::pin_states[pin].isr_mode == INT_EDGE_FALLING && !(Test_IO::pin_states[pin].current_signal == 1.0f && Test_IO::pin_states[pin].previous_signal == 0.0f)) {
        return;
    }
    if (Test_IO::pin_states[pin].isr_mode == INT_EDGE_BOTH && Test_IO::pin_states[pin].current_signal == Test_IO::pin_states[pin].previous_signal) {
        return;
    }
    std::thread isr_thread(Test_IO::pin_states[pin].isr_function);
    isr_thread.detach(); // Detach the thread to allow it to run independently
    LOG_INFO() << "ISR triggered on pin " << Test_IO::ORANGE << pin << Test_IO::RESET
             << " (" << Test_IO::ORANGE << Test_IO::getPinName(pin) << Test_IO::RESET
             << ") with mode " << Test_IO::ORANGE << isrModeToString(Test_IO::pin_states[pin].isr_mode) << Test_IO::RESET << ".";
}

void Test_IO::set_pin(int pin, int value){
    Test_IO::pin_states[pin].previous_signal = Test_IO::pin_states[pin].current_signal; // Update previous signal
    Test_IO::pin_states[pin].current_signal = value;
    isr_sim_update(pin); 
    if(MainWindow::s_instance)
        MainWindow::s_instance->updateTable(pin);
}

float Test_IO::read_pin(int pin){
    return Test_IO::pin_states[pin].current_signal;
}

// float Test_Test_IO::get_current_motor_speed(){
//     // return current_motor_speed;
// }

// int Test_Test_IO::get_current_direction(){
//     return current_direction;
// }

void Test_IO::check_wiringPiSetup(){
    if (!wiringpi_setup_called) {
        LOG_ERROR() <<  "Error: wiringPiSetup must be called before." ;
    }
}

void Test_IO::check_Pin(int pin, std::string function_name) {
    if (std::find(addressable_gpio_pins.begin(), addressable_gpio_pins.end(), pin) == addressable_gpio_pins.end()) {
        LOG_ERROR() <<  "Error: Pin " << Test_IO::ORANGE << pin << Test_IO::RESET << " is not a valid GPIO pin " << " in " << function_name << " ." ;
    }
    if (Test_IO::pin_states.find(pin) == Test_IO::pin_states.end()) {
        LOG_ERROR() <<  "Error: pinMode not set for pin " << Test_IO::ORANGE << pin << Test_IO::RESET << " in " << function_name << " ." ;
    }
    if (Test_IO::pin_states[pin].pin_mode != INPUT && function_name == "PullUpDnControl") {
        LOG_ERROR() <<  "Error: " << function_name << " attempted on pin " << Test_IO::ORANGE << pin << Test_IO::RESET << " not configured as INPUT." ;
    }
    if (Test_IO::pin_states[pin].pin_mode != OUTPUT && function_name == "digitalWrite") {
        LOG_ERROR() <<  "Error: " << function_name << " attempted on pin " << Test_IO::ORANGE << pin << Test_IO::RESET << " not configured as OUTPUT." ;
    }
    if (Test_IO::pin_states[pin].pin_mode != PWM_OUTPUT && function_name == "pwmWrite") {
        LOG_ERROR() <<  "Error: " << function_name << " attempted on pin " << Test_IO::ORANGE << pin << Test_IO::RESET << " not configured as PWM_OUTPUT." ;
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

std::string Test_IO::isrfuntionNameToString(void (*function)(void)) {
#ifdef _WIN32
    HANDLE process = GetCurrentProcess();
    static bool symInitialized = [] {
        return SymInitialize(GetCurrentProcess(), nullptr, TRUE);
    }();
    if (!symInitialized) return "Unknown function";
    DWORD64 address = reinterpret_cast<DWORD64>(function);
    char buffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME * sizeof(TCHAR)] = {};
    auto symbol = reinterpret_cast<PSYMBOL_INFO>(buffer);
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO); symbol->MaxNameLen = MAX_SYM_NAME;
    return SymFromAddr(process, address, nullptr, symbol) ? std::string(symbol->Name) : "Unknown function";
#else
    return "Function name not available";
#endif
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
