//! @file main.cpp
//! @author Noah
//! Test-Code für eine Torsteuerung mit Vorwärts- und Rückwärtsfunktion,
//! Endschalterauswertung und sanftem Beschleunigen/Abbremsen.
//! @version 0.2
//! @date 2025-01-12

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <Initialize.h>
#include <wiringPi.h>

using namespace std::chrono_literals;

namespace Noah::Engineering {
struct Pins {
  static constexpr int PWM       = 1;   // Hardware PWM
  static constexpr int DIRECTION = 4;   // Direction Control
  static constexpr int LEFT_END  = 21;  // Left End Switch
  static constexpr int RIGHT_END = 22;  // Right End Switch
  static constexpr int OPEN      = 3;   // Open Remote Button
  static constexpr int CLOSE     = 2;   // Close Remote Button
};

struct RampDelays {
  static constexpr std::chrono::milliseconds START_DURATION = 5ms;
  static constexpr std::chrono::milliseconds STOP_DURATION  = 4ms;
};

class Motor {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
public:
  /* Types */
  /*------------------------------------------------------------------------------------------------------------------*/
  enum class Status {
    NONE,
    LEFT_BLOCKED,
    RIGHT_BLOCKED,
  };

  /* Constructors / Destructor */
  /*------------------------------------------------------------------------------------------------------------------*/
  Motor()             = default;
  Motor(const Motor&) = delete;
  Motor(Motor&&)      = delete;
  ~Motor()            = default;


  /* Operators */
  /*------------------------------------------------------------------------------------------------------------------*/
  Motor& operator=(const Motor&) = delete;
  Motor& operator=(Motor&&)      = delete;


  /* Controller */
  /*------------------------------------------------------------------------------------------------------------------*/
  //! Initialisiert GPIO-Pins, Interrupts und PWM.
  void Initialize() {
    // WiringPi initialisieren
    if (wiringPiSetup() == -1) {
        std::cerr << "wiringPiSetup() schlug fehl.\n";
        std::exit(1);
    }

    // PWM-Pin konfigurieren
    pinMode(Pins::PWM, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(1024);
    pwmSetClock(1);

    // Richtungspin konfigurieren
    pinMode(Pins::DIRECTION, OUTPUT);

    // Endschalter-Pins als Eingänge, mit Pull-ups
    pinMode(Pins::LEFT_END, INPUT);
    pinMode(Pins::RIGHT_END, INPUT);
    pullUpDnControl(Pins::LEFT_END, PUD_UP);
    pullUpDnControl(Pins::RIGHT_END, PUD_UP);

    // Taster-Pins
    pinMode(Pins::OPEN, INPUT);
    pinMode(Pins::CLOSE, INPUT);
    pullUpDnControl(Pins::OPEN, PUD_UP);
    pullUpDnControl(Pins::CLOSE, PUD_UP);

    // Interrupts einrichten
    wiringPiISR(Pins::LEFT_END, INT_EDGE_BOTH, &Motor::OnIsrLeftEndStopped);
    wiringPiISR(Pins::RIGHT_END, INT_EDGE_BOTH, &Motor::OnIsrRightEndStopped);
  }

  void StopHard() {
    pwmWrite(Pins::PWM, 0);
    _currentPwmValue = 0;
  }

  void StopSlowly() {
    while (_currentPwmValue != 0) {
      _currentPwmValue += (_currentPwmValue < 0) ? 1 : -1;
      pwmWrite(Pins::PWM, std::abs(_currentPwmValue));
      std::this_thread::sleep_for(RampDelays::STOP_DURATION);
    }
    pwmWrite(Pins::PWM, 0);
    digitalWrite(Pins::DIRECTION, LOW); // optionales Zurücksetzen der Richtung
  }

  //! Setzt die Motorgeschwindigkeit mit sanftem Hoch-/Runterfahren.
  //! @param speed Gewünschte Geschwindigkeit (-1023 bis +1023).
  //!              < 0 => rückwärts, > 0 => vorwärts, == 0 => stop.
  void UpdateSpeed(int speed) {
    // Endschalter-Blockade prüfen:
    if (speed > 0 && _endStopStatus == Status::LEFT_BLOCKED) return;
    if (speed < 0 && _endStopStatus == Status::RIGHT_BLOCKED) return;

    if (speed == 0) {
      StopSlowly();
      return;
    }

    // Motor in kleinen Schritten hoch- oder runterfahren
    while (_currentPwmValue != speed) {
      // Einen Schritt Richtung Zielgeschwindigkeit
      if (_currentPwmValue < speed) _currentPwmValue++;
      else if (_currentPwmValue > speed) _currentPwmValue--;

      // Falls wir genau 0 durchqueren, Richtung setzen
      if (_currentPwmValue == 0) digitalWrite(Pins::DIRECTION, (speed >= 0) ? LOW : HIGH);

      // PWM-Wert setzen (nur Betrag)
      pwmWrite(Pins::PWM, std::abs(_currentPwmValue));
      std::this_thread::sleep_for(RampDelays::START_DURATION);
    }
  }


  /* Getter / Setter */
  /*------------------------------------------------------------------------------------------------------------------*/
  static Motor& Get() {
    static Motor result;
    return result;
  }

  [[nodiscard]] int GetCurrentSpeed() const { return _currentPwmValue; }
  [[nodiscard]] Status GetEndStopStatus() const { return _endStopStatus.load(); }


  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Events //////////////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
private:
  /* ISR */
  /*------------------------------------------------------------------------------------------------------------------*/
    
    static void OnIsrLeftEndStopped() {
    auto& motor = Motor::Get();
    if (digitalRead(Pins::LEFT_END) == LOW) {
      motor._endStopStatus = Status::LEFT_BLOCKED;
      motor.StopHard();
    } else {
      motor._endStopStatus = Status::NONE;
    }
  }
  static void OnIsrRightEndStopped() {
    auto& motor = Motor::Get();
    if (digitalRead(Pins::RIGHT_END) == LOW) {
        motor._endStopStatus = Status::RIGHT_BLOCKED;
        motor.StopHard();
    } else {
        motor._endStopStatus = Status::NONE;
    }
  }


  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
private:
  /* Variables */
  /*------------------------------------------------------------------------------------------------------------------*/
  int _currentPwmValue = 0; // negativ - rückwärts | positiv - vorwärts

  std::atomic<Status> _endStopStatus { Status::NONE };
};
}


// --------------------------------------------------
// Hauptprogramm
// --------------------------------------------------
int main() {
    using namespace Noah::Engineering;
    using namespace std::chrono_literals;

    // Motor Initialisieren
    auto& motor = Motor::Get();
    motor.Initialize();
    /*
	/Intterrups einrichten
    // Attach Interrupts (Replace nullptr with actual ISR functions)
    wiringPiISR(ControlSystem::Pin::LEFT_END, INT_EDGE_RISING, OnIsrLeftEndStopped());
    wiringPiISR(ControlSystem::Pin::RIGHT_END, INT_EDGE_RISING, OnIsrRightEndStopped());
    wiringPiISR(ControlSystem::Pin::LIGHT_Barrier, INT_EDGE_RISING, nullptr);
    */
    while (true) {
      enum class Action {
        NONE,
        OPEN,
        CLOSE,
      };
      static constexpr std::chrono::milliseconds DEBOUNCE_DELAY_DURATION = 200ms;

      // Überprüfen der angeforderten Aktion

      Action action = Action::NONE;
      if (digitalRead(Pins::OPEN) == LOW) {
        action = (action == Action::OPEN) ? Action::NONE : Action::OPEN; // Umschalten
        std::this_thread::sleep_for(DEBOUNCE_DELAY_DURATION);
      }
      else if (digitalRead(Pins::CLOSE) == LOW) {
        action = (action == Action::CLOSE) ? Action::NONE : Action::CLOSE; // Umschalten
        std::this_thread::sleep_for(DEBOUNCE_DELAY_DURATION);
      }

      // Ausführen
      switch (action) {
        case Action::OPEN:  motor.UpdateSpeed(1000);  break;
        case Action::CLOSE: motor.UpdateSpeed(-1000); break;
        default:            motor.StopSlowly();       break;
      }

      std::this_thread::sleep_for(10ms); // Minimale Wartezeit, um CPU-Last zu schonen
    }

    return 0;
}
