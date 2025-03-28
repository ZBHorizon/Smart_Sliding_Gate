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
 * @brief The Test_IO class simulates the wiringPi functions for testing the sliding gate control.
 *
 * It checks that wiringPiSetup is called before any other operation,
 * that pinMode is set before reading or writing a pin,
 * and that only valid values and modes are used.
 */
class Test_IO {
public:
// static struct wiringPiNodeStruct* wiringPiFindNode(int pin);
// static struct wiringPiNodeStruct* wiringPiNewNode(int pinBase, int numPins);
// static void wiringPiVersion(int *major, int *minor);
// static int wiringPiGlobalMemoryAccess(void);
// static int wiringPiUserLevelAccess(void);
static int wiringPiSetup(void);
// static int wiringPiSetupSys(void);
// static int wiringPiSetupGpio(void);
// static int wiringPiSetupPhys(void);
// static int wiringPiSetupPinType(enum WPIPinType pinType);
// irtual int wiringPiSetupGpioDevice(enum WPIPinType pinType);
// static void pinModeAlt(int pin, int mode);
// static enum WPIPinAlt getPinModeAlt(int pin);
static void pinMode(int pin, int mode);
static void pullUpDnControl(int pin, int pud);
static int digitalRead(int pin);
static void digitalWrite(int pin, int value);
// static unsigned int digitalRead8(int pin);
// static void digitalWrite8(int pin, int value);
static void pwmWrite(int pin, int value);
// static int analogRead(int pin);
// static void analogWrite(int pin, int value);
// static int wiringPiSetupPiFace(void);
// static int wiringPiSetupPiFaceForGpioProg(void);
// static int piGpioLayout(void);
// static int piBoardRev(void);
// static void piBoardId(int *model, int *rev, int *mem, int *maker, int *overVolted);
// static int piBoard40Pin(void);
// static int piRP1Model(void);
// static int wpiPinToGpio(int wpiPin);
// static int physPinToGpio(int physPin);
// static void setPadDrive(int group, int value);
// static void setPadDrivePin(int pin, int value);
// static int getAlt(int pin);
// static void pwmToneWrite(int pin, int freq);
static void pwmSetMode(int mode);
static void pwmSetRange(unsigned int range);
static void pwmSetClock(int divisor);
// static void gpioClockSet(int pin, int freq);
// static unsigned int digitalReadByte(void);
// static unsigned int digitalReadByte2(void);
// static void digitalWriteByte(int value);
// static void digitalWriteByte2(int value);
static int waitForInterrupt(int pin, int mS);
static int wiringPiISR(int pin, int mode, void (*function)(void));
static int wiringPiISRStop(int pin);
static int waitForInterruptClose(int pin);
// static int piThreadCreate(void* (*fn)(void *));
// static void piLock(int key);
// static void piUnlock(int key);
// static int piHiPri(const int pri);
static void delay(unsigned int howLong);
static void delayMicroseconds(unsigned int howLong);
// static unsigned int millis(void);
// static unsigned int micros(void);
// static unsigned long long piMicros64(void);
// static int wiringPiI2CRead           (int fd) ;
// static int wiringPiI2CReadReg8       (int fd, int reg) ;
static int wiringPiI2CReadReg16      (int fd, int reg) ;
// static int wiringPiI2CReadBlockData  (int fd, int reg, uint8_t *values, uint8_t size);  //Interface 3.3
// static int wiringPiI2CRawRead        (int fd, uint8_t *values, uint8_t size);           //Interface 3.3

// static int wiringPiI2CWrite          (int fd, int data) ;
// static int wiringPiI2CWriteReg8      (int fd, int reg, int data) ;
static int wiringPiI2CWriteReg16     (int fd, int reg, int data) ;
// static int wiringPiI2CWriteBlockData (int fd, int reg, const uint8_t *values, uint8_t size);  //Interface 3.3
// static int wiringPiI2CRawWrite       (int fd, const uint8_t *values, uint8_t size);           //Interface 3.3

// static int wiringPiI2CSetupInterface (const char *device, int devId) ;
static int wiringPiI2CSetup          (const int devId) ;

static void set_input(int pin, int value);


// //! Getters for simulation status.
// static float get_current_motor_speed();
// static int get_current_direction();

private:
    inline static bool wiringpi_setup_called = false; //!< Flag to check if wiringPiSetup was called.



    static void check_wiringPiSetup();
    static void check_Pin(int pin, std::string function_name);
    static std::string pinModeToString(int mode);
    static std::string pullUpToString(int pud);
    static std::string digitalValToString(int value);
    static std::string pwmModeToString(int mode);
    static std::string isrModeToString(int mode);
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
    struct _global_PWM{
        inline static int PWM_MODE = -1;
        inline static int PWM_RANGE = -1;
        inline static int PWM_CLOCK = -1;
    };

    /**
     * @brief The addressable_gpio_pins vector holds the GPIO pins that can be addressed.
     */
    inline static const std::vector<int> addressable_gpio_pins = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
    
    /**
     * @brief The addressable_pwm_pins vector holds the GPIO pins that can be used for PWM.
     */
    inline static const std::vector<int> addressable_pwm_pins = {1, 24, 28, 29,};

    static std::mutex mtx; //!< Mutex for thread safety.
    
};
} // namespace SlidingGate
