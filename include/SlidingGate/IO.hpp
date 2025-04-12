//! @file IO.hpp
//! @brief Provides IO for motor control, lamps, and sensors.

#pragma once

#ifndef TRUE
#  define TRUE  (1 == 1)
#  define FALSE (!TRUE)
#endif

// Pin modes

#define INPUT            0
#define OUTPUT           1
#define PWM_OUTPUT       2
#define PWM_MS_OUTPUT    8
#define PWM_BAL_OUTPUT   9
#define GPIO_CLOCK       3
#define SOFT_PWM_OUTPUT  4
#define SOFT_TONE_OUTPUT 5
#define PWM_TONE_OUTPUT  6
#define PM_OFF           7 // to input / release line

#define LOW              0
#define HIGH             1

// Pull up/down/none

#define PUD_OFF          0
#define PUD_DOWN         1
#define PUD_UP           2

// PWM

#define PWM_MODE_MS      0
#define PWM_MODE_BAL     1

// Interrupt levels

#define INT_EDGE_SETUP   0
#define INT_EDGE_FALLING 1
#define INT_EDGE_RISING  2
#define INT_EDGE_BOTH    3


namespace SlidingGate {
class IO {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/

public:

  // static struct wiringPiNodeStruct* wiringPiFindNode(int pin);
  // static struct wiringPiNodeStruct* wiringPiNewNode(int pinBase, int numPins);
  // static void wiringPiVersion(int *major, int *minor);
  // static int wiringPiGlobalMemoryAccess(void);
  // static int wiringPiUserLevelAccess(void);
  static int  wiringPiSetup(void);
  // static int wiringPiSetupSys(void);
  // static int wiringPiSetupGpio(void);
  // static int wiringPiSetupPhys(void);
  // static int wiringPiSetupPinType(enum WPIPinType pinType);
  // irtual int wiringPiSetupGpioDevice(enum WPIPinType pinType);
  // static void pinModeAlt(int pin, int mode);
  // static enum WPIPinAlt getPinModeAlt(int pin);
  static void pinMode(int pin, int mode);
  static void pullUpDnControl(int pin, int pud);
  static int  digitalRead(int pin);
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
  static int  waitForInterrupt(int pin, int mS);
  static int  wiringPiISR(int pin, int mode, void (*function)(void));
  static int  wiringPiISRStop(int pin);
  static int  waitForInterruptClose(int pin);
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
  static int  wiringPiI2CReadReg16(int fd, int reg);
  // static int wiringPiI2CReadBlockData  (int fd, int reg, uint8_t *values, uint8_t size);  //Interface 3.3
  // static int wiringPiI2CRawRead        (int fd, uint8_t *values, uint8_t size);           //Interface 3.3

  // static int wiringPiI2CWrite          (int fd, int data) ;
  // static int wiringPiI2CWriteReg8      (int fd, int reg, int data) ;
  static int wiringPiI2CWriteReg16(int fd, int reg, int data);
  // static int wiringPiI2CWriteBlockData (int fd, int reg, const uint8_t *values, uint8_t size);  //Interface 3.3
  // static int wiringPiI2CRawWrite       (int fd, const uint8_t *values, uint8_t size);           //Interface 3.3

  // static int wiringPiI2CSetupInterface (const char *device, int devId) ;
  static int wiringPiI2CSetup(const int devId);
};
} // namespace SlidingGate
