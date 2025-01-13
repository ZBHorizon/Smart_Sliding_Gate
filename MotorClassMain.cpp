/**
 * @file main.cpp
 * @author Noah
 * @brief Test-Code für eine Torsteuerung mit Vorwärts- und Rückwärtsfunktion,
 *        Endschalterauswertung und sanftem Beschleunigen/Abbremsen.
 * @version 0.2
 * @date 2025-01-12
 */

#include <wiringPi.h>
#include <atomic>
#include <cstdlib>    // für std::abs
#include <iostream>   // optional, falls du debug-Ausgaben möchtest

// --------------------------------------------------
// Konstante Pin-Definitionen und Rampenwerte
// --------------------------------------------------
constexpr int kPwmPin       = 1;   // Hardware PWM
constexpr int kDirPin       = 4;   // Direction Control
constexpr int kEndLeftPin   = 21;  // Left End Switch
constexpr int kEndRightPin  = 22;  // Right End Switch
constexpr int kOpenPin      = 3;   // Open Remote Button
constexpr int kClosePin     = 2;   // Close Remote Button

// Rampenverzögerungen (Millisekunden)
constexpr int kStartRampMs  = 5;
constexpr int kStopRampMs   = 4;

// --------------------------------------------------
// MotorController-Klasse
// --------------------------------------------------
class MotorController
{
public:
    MotorController() : currentPwmValue_(0), endStopStatus_(0) {}

    /**
     * @brief Initialisiert GPIO-Pins, Interrupts und PWM.
     */
    void init()
    {
        // WiringPi initialisieren
        if (wiringPiSetup() == -1) {
            std::cerr << "wiringPiSetup() schlug fehl.\n";
            std::exit(1);
        }

        // PWM-Pin konfigurieren
        pinMode(kPwmPin, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(1024);
        pwmSetClock(1);

        // Richtungspin konfigurieren
        pinMode(kDirPin, OUTPUT);

        // Endschalter-Pins als Eingänge, mit Pull-ups
        pinMode(kEndLeftPin, INPUT);
        pinMode(kEndRightPin, INPUT);
        pullUpDnControl(kEndLeftPin, PUD_UP);
        pullUpDnControl(kEndRightPin, PUD_UP);

        // Taster-Pins
        pinMode(kOpenPin, INPUT);
        pinMode(kClosePin, INPUT);
        pullUpDnControl(kOpenPin, PUD_UP);
        pullUpDnControl(kClosePin, PUD_UP);

        // Interrupts einrichten
        wiringPiISR(kEndLeftPin, INT_EDGE_BOTH, &MotorController::onLeftEndStopStatic);
        wiringPiISR(kEndRightPin, INT_EDGE_BOTH, &MotorController::onRightEndStopStatic);
    }

    // --------------------------------------------------
    // Motor-Aktionen
    // --------------------------------------------------
    /**
     * @brief Stoppt den Motor sofort (Hard Stop).
     */
    void stopMotor()
    {
        pwmWrite(kPwmPin, 0);
        currentPwmValue_ = 0;
    }

    /**
     * @brief Führt einen sanften Stopp durch, also schrittweises Absenken von speed bis 0.
     */
    void softStop()
    {
        while (currentPwmValue_ != 0) {
            currentPwmValue_ += currentPwmValue_ < 0) ? 1 : -1;
            pwmWrite(kPwmPin, std::abs(currentPwmValue_));
            delay(kStopRampMs);
        }
        pwmWrite(kPwmPin, 0);
        digitalWrite(kDirPin, LOW); // optionales Zurücksetzen der Richtung
    }

    /**
     * @brief Setzt die Motorgeschwindigkeit mit sanftem Hoch-/Runterfahren.
     * @param speed Gewünschte Geschwindigkeit (-1023 bis +1023).
     *              < 0 => rückwärts, > 0 => vorwärts, == 0 => stop.
     */
    void setMotorSpeed(int speed)
    {
        // Endschalter-Blockade prüfen:
        // endStopStatus_ == -1 => links blockiert,
        // endStopStatus_ == 1  => rechts blockiert.
        if (speed > 0 && endStopStatus_ == -1) return; // links blockiert
        if (speed < 0 && endStopStatus_ == 1)  return; // rechts blockiert

        // 0 => sanfter Stopp
        if (speed == 0) {
            softStop();
            return;
        }

        // Motor in kleinen Schritten hoch- oder runterfahren
        while (currentPwmValue_ != speed)
        {
            // Einen Schritt Richtung Zielgeschwindigkeit
            if (currentPwmValue_ < speed) currentPwmValue_++;
            else if (currentPwmValue_ > speed) currentPwmValue_--;

            // Falls wir genau 0 durchqueren, Richtung setzen
            if (currentPwmValue_ == 0) {
                digitalWrite(kDirPin, (speed >= 0) ? LOW : HIGH);
            }

            // PWM-Wert setzen (nur Betrag)
            pwmWrite(kPwmPin, std::abs(currentPwmValue_));
            delay(kStartRampMs);
        }
    }

    // --------------------------------------------------
    // Getter (z.B. für Debug oder Taster-Abfrage)
    // --------------------------------------------------
    int  getCurrentSpeed() const { return currentPwmValue_; }
    int  getEndStopStatus() const { return endStopStatus_.load(); }

    // --------------------------------------------------
    // ISR-Funktionen (werden statisch aufgerufen)
    // --------------------------------------------------
    static void onLeftEndStopStatic()
    {
        // Globale Referenz auf *ein* MotorController-Objekt
        // => Ruft dann die nicht-statische Methode auf
        if (instancePtr_) instancePtr_->onLeftEndStop();
    }

    static void onRightEndStopStatic()
    {
        if (instancePtr_) instancePtr_->onRightEndStop();
    }

    // --------------------------------------------------
    // Setter für die globale Instanz, damit ISR funktioniert
    // --------------------------------------------------
    static void setInstancePtr(MotorController* ptr)
    {
        instancePtr_ = ptr;
    }

private:
    /**
     * @brief ISR-Callback für den linken Endschalter.
     */
    void onLeftEndStop()
    {
        if (digitalRead(kEndLeftPin) == LOW)
        {
            endStopStatus_ = -1; // links blockiert
            stopMotor();
        }
        else
        {
            endStopStatus_ = 0;  // freigegeben
        }
    }

    /**
     * @brief ISR-Callback für den rechten Endschalter.
     */
    void onRightEndStop()
    {
        if (digitalRead(kEndRightPin) == LOW)
        {
            endStopStatus_ = 1; // rechts blockiert
            stopMotor();
        }
        else
        {
            endStopStatus_ = 0; // freigegeben
        }
    }

private:
    // Aktuelle PWM-Geschwindigkeit (negativ = rückwärts, positiv = vorwärts)
    int currentPwmValue_;

    // Endschalter-Status: -1 = links blockiert, 0 = keiner, 1 = rechts blockiert
    std::atomic<int> endStopStatus_;

    // Statische Zeiger, um in ISRs auf dieses Objekt zugreifen zu können
    static MotorController* instancePtr_;
};

// Definition des statischen Zeigers (muss einmalig im .cpp stehen)
MotorController* MotorController::instancePtr_ = nullptr;


// --------------------------------------------------
// Hauptprogramm
// --------------------------------------------------
int main()
{
    // Motorcontroller anlegen und Initialisieren
    MotorController motor;
    MotorController::setInstancePtr(&motor); // Damit ISR-Aufrufe wissen, auf welches Objekt sie zugreifen sollen
    motor.init();

    // Taster-Abfrage
    int buttonPressed = -1;         // -1 = keine Taste, 0 = Open, 1 = Close
    constexpr int debounceDelayMs = 200;

    while (true)
    {
        // Open-Taste?
        if (digitalRead(kOpenPin) == LOW) {
            // Umschalten zwischen Start/Stopp
            buttonPressed = (buttonPressed == 0) ? -1 : 0;
            delay(debounceDelayMs);
        }
        // Close-Taste?
        else if (digitalRead(kClosePin) == LOW) {
            buttonPressed = (buttonPressed == 1) ? -1 : 1;
            delay(debounceDelayMs);
        }

        // Aktionen ausführen
        switch (buttonPressed)
        {
            case 0:
                // Vorwärts => setMotorSpeed(+1000)
                motor.setMotorSpeed(1000);
                break;
            case 1:
                // Rückwärts => setMotorSpeed(-1000)
                motor.setMotorSpeed(-1000);
                break;
            case -1:
            default:
                // Keine Taste => sanft stoppen
                motor.softStop();
                break;
        }

        delay(10); // Minimale Wartezeit, um CPU-Last zu schonen
    }

    return 0;
}
