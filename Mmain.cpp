/**
 * @file main.cpp
 * @author Noah
 * @brief  Test Code um ein Tor öffnen und schließen zu könnnen
 * @version 0.1
 * @date 2025-01-12
 * @copyright Copyright (c) 2025
 * 
 * Zustandstabelle:
 * +-----------------------------------------+-------------------+-----------+-----------------------------+------------------------------+---------------------------------------------------------------------------------+
 * | Zustand                                 | ButtonPressed     | Limit     | Endschalter Links aktiv     | Endschalter Rechts aktiv     | Motoraktion                                                                      |
 * +-----------------------------------------+-------------------+-----------+-----------------------------+------------------------------+---------------------------------------------------------------------------------+
 * | Keine Taste gedrückt                    | -1                | 0         | Nein                        | Nein                         | Motor führt einen sanften Stopp aus (ruht, falls bereits gestoppt).             |
 * | Open-Taste gedrückt                     | 0                 | 0         | Nein                        | Nein                         | Motor bewegt sich vorwärts mit Geschwindigkeit 500.                             |
 * | Close-Taste gedrückt                    | 1                 | 0         | Nein                        | Nein                         | Motor bewegt sich rückwärts mit Geschwindigkeit -500.                           |
 * | Open-Taste gedrückt                     | 0                 | -1        | Ja                          | Nein                         | Bewegung wird blockiert (vorwärts nicht erlaubt).                               |
 * | Close-Taste gedrückt                    | 1                 | 1         | Nein                        | Ja                           | Bewegung wird blockiert (rückwärts nicht erlaubt).                              |
 * | Open-Taste gedrückt                     | 0                 | 1         | Nein                        | Ja                           | Motor bewegt sich vorwärts, da nur rückwärts blockiert ist.                     |
 * | Close-Taste gedrückt                    | 1                 | -1        | Ja                          | Nein                         | Motor bewegt sich rückwärts, da nur vorwärts blockiert ist.                     |
 * | Keine Taste gedrückt (links blockiert)  | -1                | -1        | Ja                          | Nein                         | Motor bleibt gestoppt (links blockiert).                                        |
 * | Keine Taste gedrückt (rechts blockiert) | -1                | 1         | Nein                        | Ja                           | Motor bleibt gestoppt (rechts blockiert).                                       |
 * | Gleicher Taster erneut gedrückt         | 0 oder 1          | 0         | Nein                        | Nein                         | Motor führt einen sanften Stopp aus.                                            |
 * | Anderer Taster während der Bewegung     | 0 oder 1          | 0         | Nein                        | Nein                         | Motor wechselt die Richtung und passt Geschwindigkeit an (weiche Transition).   |
 * +-----------------------------------------+-------------------+-----------+-----------------------------+------------------------------+---------------------------------------------------------------------------------+
 * ButtonPressed:
 * -1: Keine Taste gedrückt.
 * 0: Open-Taste gedrückt.
 * 1: Close-Taste gedrückt.
 * Limit: 
 * 0: Kein Endlagensensor aktiv.
 * -1: Links-Endschalter ausgelöst.
 * 1: Rechts-Endschalter ausgelöst.
 */
#include <wiringPi.h>
#include <stdbool.h>
#include <stdlib.h> // Für abs()

#define PWM_PIN 1       //  (Hardware PWM)
#define DIR_PIN_1 4     //  (Motor Direction Control)
#define END_LEFT_PIN 21  //  (Left End Switch)
#define END_RIGHT_PIN 22 //  (Right End Switch)
#define OPEN_PIN 3  // (Open Remote Button)
#define CLOSE_PIN 2 //  (Close Remote Button)

const int start_ramp = 5; // Verzögerung für Ramp-up in Millisekunden
const int stop_ramp = 4;   // Verzögerung für Ramp-down in Millisekunden

int current_speed = 0;        // Verfolgt die aktuelle Geschwindigkeit des Motors
volatile int limit = 0; // Flag zur Anzeige einer Stoppanforderung

/**
 * @brief Initialisiert die GPIO-Pins und konfiguriert die Interrupt-Handler.
 */
void setup() {
    // Initialisiere die WiringPi-Bibliothek
    if (wiringPiSetup() == -1) { // Rückgabewert -1 bedeutet, dass die Initialisierung fehlgeschlagen ist.
        exit(1); // Programm abbrechen
    }
    // PWM-Pin konfigurieren
    pinMode(PWM_PIN, PWM_OUTPUT); // Aktiviere Hardware-PWM
    pwmSetMode(PWM_MODE_MS);      // Setze Mark-Space-Modus für stabile Frequenz
    pwmSetRange(1024);            // Setze den PWM-Bereich auf 1024
    pwmSetClock(1);               // Setze den Taktteiler auf 1 für die PWM-Frequenz

    // Richtungspin konfigurieren
    pinMode(DIR_PIN_1, OUTPUT);

    // Endschalter als Eingänge konfigurieren und Pull-ups aktivieren
    pinMode(END_LEFT_PIN, INPUT);
    pinMode(END_RIGHT_PIN, INPUT);
    pullUpDnControl(END_LEFT_PIN, PUD_UP);
    pullUpDnControl(END_RIGHT_PIN, PUD_UP);

    pinMode(OPEN_PIN, INPUT);
    pinMode(CLOSE_PIN, INPUT);
    pullUpDnControl(OPEN_PIN, PUD_UP);
    pullUpDnControl(CLOSE_PIN, PUD_UP);
}
/**
 * @brief Stoppt den Motor sofort.
 */
void stopMotor() {
    pwmWrite(PWM_PIN, 0);       // PWM-Signal deaktivieren
    current_speed = 0; // Setze Geschwindigkeit auf 0
}
void interrupt() {
    // Links-Endschalter
    wiringPiISR(END_LEFT_PIN, INT_EDGE_BOTH, []() {
        if (digitalRead(END_LEFT_PIN) == LOW) {
            limit = -1; // Links-Endschalter ausgelöst
            stopMotor();
        } else {
            limit = 0;  // Links-Endschalter freigegeben
        }
    });
    // Rechts-Endschalter
    wiringPiISR(END_RIGHT_PIN, INT_EDGE_BOTH, []() {
        if (digitalRead(END_RIGHT_PIN) == LOW) {
            limit = 1; // Rechts-Endschalter ausgelöst
            stopMotor();
        } else {
            limit = 0;  // Rechts-Endschalter freigegeben
        }
    });
}
/**
 * @brief Führt einen sanften Stopp aus, indem die Geschwindigkeit langsam auf 0 reduziert wird.
 */
void soft_stop() {
    // Geschwindigkeit sanft reduzieren
    while (current_speed != 0) {
        current_speed += (current_speed < 0) ? 1 : -1;
        pwmWrite(PWM_PIN, abs(current_speed));
        delay(stop_ramp); // Sanfter Übergang
    }
    pwmWrite(PWM_PIN, 0);       // PWM-Signal deaktivieren
    digitalWrite(DIR_PIN_1, LOW); // Richtung zurücksetzen
}

/**
 * @brief Setzt die Motorgeschwindigkeit mit sanftem Hoch- und Herunterfahren.
 * 
 * @param speed Zielgeschwindigkeit: 
 * - Positive Werte -> Vorwärtsbewegung.
 * - Negative Werte -> Rückwärtsbewegung.
 * - `0` -> Motor anhalten.
 */
void setMotorSpeed(int speed) {
    if(speed > 0 && limit == -1) return;
    if(speed < 0 && limit == 1) return;
    
    // Falls Geschwindigkeit 0 ist, führen wir einen sanften Stopp aus
    if (speed == 0) {
        soft_stop();
        return;
    } 

    // Geschwindigkeit sanft hoch- oder runterfahren
    while (current_speed != speed) {
        //-1023 bis 1023 -> dir High bis Low und 0 bis 1023
        current_speed += (current_speed < speed) ? 1 : -1;
        if (current_speed==0){
            digitalWrite(DIR_PIN_1, speed >= 0 ? LOW : HIGH); // Richtung setzen, kleiner gleich 0 ist LOW und größer als 0 ist HIGH}
        }
        pwmWrite(PWM_PIN, abs(current_speed));
        delay(start_ramp); // Sanftes Übergangsverhalten
        }
    }
}

int main(void) {
    setup(); // Initialisiere Pins und Interrupts
    interrupt();
    int buttonPressed = -1;       // Speichert den Zustand der Tasten: -1 = keine Taste gedrückt
    const int debounceDelay = 200; // Verzögerung für Debouncing in Millisekunden

    while (true) {

        // Überprüfe, welche Taste gedrückt wurde
        if (digitalRead(OPEN_PIN) == LOW) {
            buttonPressed = (buttonPressed == 0) ? -1 : 0; // Umschalten zwischen Start/Stopp für Open
            delay(debounceDelay); // Debounce-Zeit
        } else if (digitalRead(CLOSE_PIN) == LOW) {
            buttonPressed = (buttonPressed == 1) ? -1 : 1; // Umschalten zwischen Start/Stopp für Close
            delay(debounceDelay); // Debounce-Zeit
        }

        // Handle die Aktion basierend auf der gedrückten Taste
        switch (buttonPressed) {
            case 0: // Open-Taste
                setMotorSpeed(1000); // Motor vorwärts starten
                break;

            case 1: // Close-Taste
                setMotorSpeed(-1000); // Motor rückwärts starten
                break;

            case -1: // Keine Taste gedrückt oder Stopp
                soft_stop(); // Motor sanft anhalten
                break;

            default:
                // Kein Fall erforderlich, falls andere Werte auftreten
                break;
        }

        delay(10); // Kurze Pause, um CPU-Last zu reduzieren
    }

    return 0;
}
