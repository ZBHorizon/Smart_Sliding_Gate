#include "Initialize.h"
#include <iostream>
#include <cstdlib>
/*
+----------+------+-----+----------+-----+------+----------+
|Function  |Mode  |Pin  |Physical  |Pin  |Mode  |Function  |
+----------+------+-----+----------+-----+------+----------+
|3.3V      |      |     | 1|2      |     |      |5V        |
|          |      |8    | 3|4      |     |      |5V        |
|          |      |9    | 5|6      |     |      |0V        |
|          |      |7    | 7|8      |15   |      |LAMP      |
|0V        |      |     | 9|10     |16   |      |RESERVE_OUT|
|          |      |0    | 11|12    |1    |      |PWM        |
|REMOTE_B  |      |2    | 13|14    |     |      |0V         |
|REMOTE_A  |      |3    | 15|16    |4    |      |DIRECTION  |
|3.3V      |      |     | 17|18    |5    |      |LIGHT_Barrier|
|REMOTE_C  |      |12   | 19|20    |     |      |0V         |
|REMOTE_D  |      |13   | 21|22    |6    |      |RESERVE_IN  |
|          |      |14   | 23|24    |10   |      |            |
|0V        |      |     | 25|26    |11   |      |            |
|          |      |30   | 27|28    |31   |      |            |
|LEFT_END  |      |21   | 29|30    |     |      |0V          |
|RIGHT_END |      |22   | 31|32    |26   |      |            |
|          |      |23   | 33|34    |     |      |0V          |
|          |      |24   | 35|36    |27   |      |            |
|          |      |25   | 37|38    |28   |      |            |
|0V        |      |     | 39|40    |29   |      |            |
+----------+------+-----+----------+-----+------+----------+
*/

namespace ControlSystem {
    struct Pin {
        static constexpr int PWM = 1;   // Hardware PWM
        static constexpr int DIRECTION = 4;   // Direction Control
        static constexpr int LEFT_END = 21;  // Left End Switch
        static constexpr int RIGHT_END = 22;  // Right End Switch
        static constexpr int REMOTE_A = 3;   // Open Remote Button
        static constexpr int REMOTE_B = 2;   // Half Open Remote Button
        static constexpr int REMOTE_C = 12;   // Close Remote Button
        static constexpr int REMOTE_D = 13;   // Garden Door Remote Button
		static constexpr int LAMP = 15;   // Gate Lamp
        static constexpr int RESERVE_OUT = 16;
		static constexpr int LIGHT_Barrier = 5;   // Light Barrier Sensor
        static constexpr int RESERVE_IN = 6;   
    };

    void InitializeGPIO() {
        // Initialize WiringPi
        if (wiringPiSetup() == -1) {
            std::cerr << "Failed to initialize WiringPi.\n";
            std::exit(1);
        }

        // Configure PWM
        pinMode(Pin::PWM, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(1024);
        pwmSetClock(1);

        // Configure Output Pins
        pinMode(Pin::DIRECTION, OUTPUT);
        pinMode(Pin::LAMP, OUTPUT);
        pinMode(Pin::RESERVE_OUT, OUTPUT);

        // Configure Input Pins with Pull-Up Resistors
        pinMode(Pin::LEFT_END, INPUT);
        pinMode(Pin::RIGHT_END, INPUT);
        pullUpDnControl(Pin::LEFT_END, PUD_UP);
        pullUpDnControl(Pin::RIGHT_END, PUD_UP);

        pinMode(Pin::REMOTE_A, INPUT);
        pinMode(Pin::REMOTE_B, INPUT);
        pinMode(Pin::REMOTE_C, INPUT);
        pinMode(Pin::REMOTE_D, INPUT);
        pullUpDnControl(Pin::REMOTE_A, PUD_UP);
        pullUpDnControl(Pin::REMOTE_B, PUD_UP);
        pullUpDnControl(Pin::REMOTE_C, PUD_UP);
        pullUpDnControl(Pin::REMOTE_D, PUD_UP);

        pinMode(Pin::LIGHT_Barrier, INPUT);
        pullUpDnControl(Pin::LIGHT_Barrier, PUD_UP);

        pinMode(Pin::RESERVE_IN, INPUT);
        pullUpDnControl(Pin::RESERVE_IN, PUD_UP);

        std::cout << "GPIO initialized successfully.\n";
    }
}
