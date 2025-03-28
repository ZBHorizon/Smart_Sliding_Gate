 /**
     * @brief The GateSimulator class simulates the physical gate movement.
     *
     * It continuously updates the gate position based on the current motor speed.
     * Positive motor speed (from pwmWrite) is interpreted as opening (increases position),
     * while negative speed indicates closing (decreases position).
     *
     * The time to fully open or close is defined by TIME_TO_OPEN and TIME_TO_CLOSE.
     */
    // class GateSimulator {
    //     public:
    
    
    //         static void simulation_loop();
    
    
    //     private:
    //         //! The simulation loop that updates the gate position.
            
    
    //         static bool running;                   //!< Flag to control simulation loop.
    
    //         // Gate simulation state and time constants.
    //         inline static float current_gate_position = 0.0f;  //!< Current gate position.
    //         static constexpr std::chrono::milliseconds TIME_TO_OPEN{26000};  //!< Time to fully open.
    //         static constexpr std::chrono::milliseconds TIME_TO_CLOSE{24000}; //!< Time to fully close.
    //     };
    
    // } // namespace SlidingGate