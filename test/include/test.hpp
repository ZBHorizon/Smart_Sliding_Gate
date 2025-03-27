#include <SlidingGate/IO.hpp>

#ifndef TEST_HPP
#define TEST_HPP

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
     * @brief The Test_IO class simulates the wiringPi functions for testing the sliding gate control.
     *
     * It checks that wiringPiSetup is called before any other operation,
     * that pinMode is set before reading or writing a pin,
     * and that only valid values and modes are used.
     */
    class Test_IO {
    public:
        

        //! Function to simulate an ISR trigger on a given pin.
         static void simulate_isr(int pin);

        // //! Getters for simulation status.
        // static float get_current_motor_speed();
        // static int get_current_direction();

    private:
        static bool wiringpi_setup_called; //!< Flag to check if wiringPiSetup was called.
        // static std::unordered_map<int, PinMode> pin_mode_map; //!< Map to store pin modes.
        static std::unordered_map<int, int> pin_state_map; //!< Map to store pin states.
        static std::unordered_map<int, std::function<void(void)>> isr_map; //!< Map to store ISR functions.
        static std::mutex mtx; //!< Mutex for thread safety.

        // For PWM simulation:
        static std::uint32_t pwm_range; //!< PWM range.
        static int pwm_mode; //!< PWM mode.
        static int pwm_clock; //!< PWM clock divisor.
        
        // Simulation variables for motor speed and direction:
        static int current_direction; // (0 = LOW, 1 = HIGH) //!< Current motor direction.
        static float current_motor_speed; //!< Current motor speed.
    };

    /**
     * @brief The GateSimulator class simulates the physical gate movement.
     *
     * It continuously updates the gate position based on the current motor speed.
     * Positive motor speed (from pwmWrite) is interpreted as opening (increases position),
     * while negative speed indicates closing (decreases position).
     *
     * The time to fully open or close is defined by TIME_TO_OPEN and TIME_TO_CLOSE.
     */
    class GateSimulator {
    public:


        static void simulation_loop();


    private:
        //! The simulation loop that updates the gate position.
        

        static bool running;                   //!< Flag to control simulation loop.

        // Gate simulation state and time constants.
        inline static float current_gate_position = 0.0f;  //!< Current gate position.
        static constexpr std::chrono::milliseconds TIME_TO_OPEN{26000};  //!< Time to fully open.
        static constexpr std::chrono::milliseconds TIME_TO_CLOSE{24000}; //!< Time to fully close.
    };

} // namespace SlidingGate

#endif // TEST_HPP
