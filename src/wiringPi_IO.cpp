#include <SlidingGate/IO.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>

namespace SlidingGate{
    
// IO::struct wiringPiNodeStruct* wiringPiFindNode(int pin) {
//     return ::wiringPiFindNode(pin);
// }
// IO::struct wiringPiNodeStruct* wiringPiNewNode(int pinBase, int numPins) {
//     return ::wiringPiNewNode(pinBase, numPins);
// }
// IO::void wiringPiVersion(int *major, int *minor) {
//     ::wiringPiVersion(major, minor);
// }
// IO::int wiringPiGlobalMemoryAccess(void) {
//     return ::wiringPiGlobalMemoryAccess();
// }
// IO::int wiringPiUserLevelAccess(void) {
//     return ::wiringPiUserLevelAccess();
// }
int IO::wiringPiSetup(void) {
    return ::wiringPiSetup();
}
// IO::int wiringPiSetupSys(void) {
//     return ::wiringPiSetupSys();
// }
// IO::int wiringPiSetupGpio(void) {
//     return ::wiringPiSetupGpio();
// }
// IO::int wiringPiSetupPhys(void) {
//     return ::wiringPiSetupPhys();
// }
// IO::int wiringPiSetupPinType(enum WPIPinType pinType) {
//     return ::wiringPiSetupPinType(pinType);
// }
// IO::int wiringPiSetupGpioDevice(enum WPIPinType pinType) {
//     return ::wiringPiSetupGpioDevice(pinType);
// }
// IO::void pinModeAlt(int pin, int mode) {
//     ::pinModeAlt(pin, mode);
// }
// IO::enum WPIPinAlt getPinModeAlt(int pin) {
//     return ::getPinModeAlt(pin);
// }
void IO::pinMode(int pin, int mode) {
    ::pinMode(pin, mode);
}
void IO::pullUpDnControl(int pin, int pud) {
    ::pullUpDnControl(pin, pud);
}
int IO::digitalRead(int pin) {
    return ::digitalRead(pin);
}
void IO::digitalWrite(int pin, int value) {
    ::digitalWrite(pin, value);
}
// IO::unsigned int digitalRead8(int pin) {
//     return ::digitalRead8(pin);
// }
// IO::void digitalWrite8(int pin, int value) {
//     ::digitalWrite8(pin, value);
// }
void IO::pwmWrite(int pin, int value) {
    ::pwmWrite(pin, value);
}
// IO::int analogRead(int pin) {
//     return ::analogRead(pin);
// }
// IO::void analogWrite(int pin, int value) {
//     ::analogWrite(pin, value);
// }
// IO::int wiringPiSetupPiFace(void) {
//     return ::wiringPiSetupPiFace();
// }
// IO::int wiringPiSetupPiFaceForGpioProg(void) {
//     return ::wiringPiSetupPiFaceForGpioProg();
// }
// int piGpioLayout(void) {
//     return ::piGpioLayout();
// }
// int piBoardRev(void) {
//     return ::piBoardRev();
// }
// void piBoardId(int *model, int *rev, int *mem, int *maker, int *overVolted) {
//     ::piBoardId(model, rev, mem, maker, overVolted);
// }
// int piBoard40Pin(void) {
//     return ::piBoard40Pin();
// }
// int piRP1Model(void) {
//     return ::piRP1Model();
// }
// int wpiPinToGpio(int wpiPin) {
//     return ::wpiPinToGpio(wpiPin);
// }
// int physPinToGpio(int physPin) {
//     return ::physPinToGpio(physPin);
// }
// void setPadDrive(int group, int value) {
//     ::setPadDrive(group, value);
// }
// void setPadDrivePin(int pin, int value) {
//     ::setPadDrivePin(pin, value);
// }
// int getAlt(int pin) {
//     return ::getAlt(pin);
// }
// void pwmToneWrite(int pin, int freq) {
//     ::pwmToneWrite(pin, freq);
// }
void IO::pwmSetMode(int mode) {
    ::pwmSetMode(mode);
}
void IO::pwmSetRange(unsigned int range) {
    ::pwmSetRange(range);
}
void IO::pwmSetClock(int divisor) {
    ::pwmSetClock(divisor);
}
// void gpioClockSet(int pin, int freq) {
//     ::gpioClockSet(pin, freq);
// }
// unsigned int digitalReadByte(void) {
//     return ::digitalReadByte();
// }
// unsigned int digitalReadByte2(void) {
//     return ::digitalReadByte2();
// }
// void digitalWriteByte(int value) {
//     ::digitalWriteByte(value);
// }
// void digitalWriteByte2(int value) {
//     ::digitalWriteByte2(value);
// }
int IO::waitForInterrupt(int pin, int mS) {
    return ::waitForInterrupt(pin, mS);
}
int IO::wiringPiISR(int pin, int mode, void (*function)(void)) {
    return ::wiringPiISR(pin, mode, function);
}
int IO::wiringPiISRStop(int pin) {
    return ::wiringPiISRStop(pin);
}
int IO::waitForInterruptClose(int pin) {
    return ::waitForInterruptClose(pin);
}
// int piThreadCreate(void* (*fn)(void *)) {
//     return ::piThreadCreate(fn);
// }
// void piLock(int key) {
//     ::piLock(key);
// }
// void piUnlock(int key) {
//     ::piUnlock(key);
// }
// int piHiPri(const int pri) {
//     return ::piHiPri(pri);
// }
void IO::delay(unsigned int howLong) {
    ::delay(howLong);
}
void IO::delayMicroseconds(unsigned int howLong) {
    ::delayMicroseconds(howLong);
}
// unsigned int millis(void) {
//     return ::millis();
// }
// unsigned int micros(void) {
//     return ::micros();
// }
// unsigned long long piMicros64(void) {
//     return ::piMicros64();
// }
// int wiringPiI2CRead(int fd) {
//     return ::wiringPiI2CRead(fd);
// }
// int wiringPiI2CReadReg8(int fd, int reg) {
//     return ::wiringPiI2CReadReg8(fd, reg);
// }
int IO::wiringPiI2CReadReg16(int fd, int reg) {
    return ::wiringPiI2CReadReg16(fd, reg);
}
// int wiringPiI2CReadBlockData(int fd, int reg, uint8_t *values, uint8_t size) {
//     return ::wiringPiI2CReadBlockData(fd, reg, values, size);
// }
// int wiringPiI2CRawRead(int fd, uint8_t *values, uint8_t size) {
//     return ::wiringPiI2CRawRead(fd, values, size);
// }
// int wiringPiI2CWrite(int fd, int data) {
//     return ::wiringPiI2CWrite(fd, data);
// }
// int wiringPiI2CWriteReg8(int fd, int reg, int data) {
//     return ::wiringPiI2CWriteReg8(fd, reg, data);
// }
int IO::wiringPiI2CWriteReg16(int fd, int reg, int data) {
    return ::wiringPiI2CWriteReg16(fd, reg, data);
}
// int wiringPiI2CWriteBlockData(int fd, int reg, const uint8_t *values, uint8_t size) {
//     return ::wiringPiI2CWriteBlockData(fd, reg, values, size);
// }
// int wiringPiI2CRawWrite(int fd, const uint8_t *values, uint8_t size) {
//     return ::wiringPiI2CRawWrite(fd, values, size);
// }
// int wiringPiI2CSetupInterface(const char *device, int devId) {
//     return ::wiringPiI2CSetupInterface(device, devId);
// }
int IO::wiringPiI2CSetup(const int devId) {
    return ::wiringPiI2CSetup(devId);
}
} // namespace SladingGate