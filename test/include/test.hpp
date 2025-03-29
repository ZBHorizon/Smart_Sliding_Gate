#pragma once

#include <SlidingGate/IO.hpp>

#include <cstdint>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>

namespace SlidingGate {
/**
 * @brief The Test_IO struct simulates the wiringPi functions for testing the sliding gate control.
 */
struct Test_IO {

    static void set_pin(int pin, int value);
    static float read_pin(int pin);
    inline static bool wiringpi_setup_called = false; //!< Flag to check if wiringPiSetup was called.

    static void check_wiringPiSetup();
    static void check_Pin(int pin, std::string function_name);
    static std::string pinModeToString(int mode);
    static std::string pullUpToString(int pud);
    static std::string digitalValToString(int value);
    static std::string pwmModeToString(int mode);
    static std::string isrModeToString(int mode);
    static std::string isrfuntionNameToString(void (*function)(void));
    static std::string getPinName(int pin);

    //! Function to simulate an ISR trigger on a given pin.
    static void isr_sim_update(int pin);
    
    /**
     * @brief The PinState struct holds the state of a GPIO pin.
     */
    struct PinState {
        float current_signal;          // HIGH(1.0f), LOW(0.0f), PWM(0.0f-1.0f <- 0V-3.3V)
        float previous_signal;         // HIGH(1.0f), LOW(0.0f), PWM(0.0f-1.0f <- 0V-3.3V)
        int pin_mode;                // INPUT, OUTPUT, PWM_OUTPUT
        int pullUpDown;              // PULL_UP, PULL_DOWN, OFF
        void (*isr_function)(void);  // Pointer to isr function
        int isr_mode;                // EDGE_RISING, EDGE_FALLING, EDGE_BOTH
    };

    /**
     * @brief The pin_states map holds the state of each GPIO pin.
     */
    inline static std::unordered_map<int, PinState> pin_states;

    /**
     * @brief The pin_mode_map holds the mode of each GPIO pin.
     */
    struct global_PWM{
        inline static int PWM_MODE = -1;
        inline static int PWM_RANGE = -1;
        inline static int PWM_CLOCK = -1;
    };

    inline static constexpr const char* ORANGE = "\033[38;5;208m";
    inline static constexpr const char* RESET  = "\033[0m";

    /**
     * @brief The addressable_gpio_pins vector holds the GPIO pins that can be addressed.
     */
    inline static const std::vector<int> addressable_gpio_pins = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
    
    /**
     * @brief The addressable_pwm_pins vector holds the GPIO pins that can be used for PWM.
     */
    inline static const std::vector<int> addressable_pwm_pins = {1, 24, 28, 29,};

    inline static std::mutex test_mtx; //!< Mutex for thread safety.
    
};
} // namespace SlidingGate
