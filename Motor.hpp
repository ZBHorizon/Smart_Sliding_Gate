/**
 * @file Motor.hpp
 * @brief Deklaration der Motor-Klasse zur Steuerung des Schiebetors.
 */

#pragma once
#include <cstdint>
#include <chrono>
using namespace std::chrono;

namespace SlidingGate {

/**
 * @brief Steuert den Motor des Schiebetors.
 */
class Motor {
public:
    /**
     * @brief Endlosschleife, in der der Motor gesteuert wird.
     */
    static void motor_loop(); 

    /**
     * @brief Führt die Kalibrierung der Öffnungs-/Schließzeiten durch.
     */
    static void calibrate_timing();

    /**
     * @brief Liest die aktuelle Geschwindigkeit des Motors.
     * @return aktuelle Geschwindigkeit
     */
    static float read_speed();

    /**
     * @brief Liest die aktuelle Position des Motors.
     * @return aktuelle Position in Prozent
     */
    static float read_position();

    /**
     * @brief Prüft, ob der Motor kalibriert ist.
     * @return true, wenn der Motor kalibriert ist
     */
    static bool is_calibrated();

    static std::condition_variable motor_cv; ///< Condition Variable für Motorsteuerung

private:
    // Interne statische Variablen
    inline static float _actual_speed = 0.0f;      ///< Aktuelle Geschwindigkeit
    inline static float _actual_position = 0.0f;     ///< Aktuelle Position (in Prozent)
    inline static milliseconds _time_to_open = 0ms;  ///< Zeit zum vollständigen Öffnen (ms)
    inline static milliseconds _time_to_close = 0ms; ///< Zeit zum vollständigen Schließen (ms)
    inline static bool _is_calibrated = false;       ///< Flag, ob Kalibrierung abgeschlossen

    static bool _overcurrent_active;                ///< Flag für Überstrom
    inline static bool _overcurrent_active = false;  ///< Initialisierung

    /**
     * @brief Zustände des Motors.
     */
    enum class MotorState {
        Opening,   ///< Tor wird geöffnet
        Closing,   ///< Tor wird geschlossen
        None       ///< Kein Vorgang
    };

    inline static MotorState _motor_state = MotorState::None; ///< Aktueller Motorzustand
    static time_point<steady_clock> _start_timestamp;    ///< Startzeitpunkt der Bewegung
    static time_point<steady_clock> _stop_timestamp;     ///< Endzeitpunkt der Bewegung
    inline static milliseconds _start_position_ms = 0ms;        ///< Startrichtung in Zeit

    /**
     * @brief Aktualisiert die aktuelle Position des Tors.
     */
    static void update_current_position();

    /**
     * @brief Prüft, ob ein Überstrom vorliegt.
     */
    static void check_for_overcurrent();

    /**
     * @brief Prüft, ob der Lichtschranken-Sensor aktiv ist.
     */
    static void check_light_barrier();

    /**
     * @brief Prüft, ob die Endschalter aktiviert sind.
     */
    static void check_end_switches();

    /**
     * @brief Aktualisiert den internen Zustand des Motors.
     */
    static void update_states();

    /**
     * @brief Setzt die Motorgeschwindigkeit.
     * @param speed Zielgeschwindigkeit.
     */
    static void set_speed(float speed);

    /**
     * @brief Aktualisiert die Motorsteuerung basierend auf Position.
     * @return true, wenn ein Fehlerzustand erkannt wurde.
     */
    static bool update_motor();

    // Überstrom-Mechanismus
    static steady_clock::time_point overcurrent_start;

    /**
     * @brief Parameter für Timing und Geschwindigkeitsregelung.
     */
    struct Param {
        inline static float calibration_speed = 0.30f;  ///< Kalibrierspeed
        static constexpr uint16_t current_threshold = 5000;  ///< Überstrom-Schwellenwert (mA)
        inline static milliseconds overcurrent_duration = 100ms; ///< Dauer zur Bestätigung des Überstroms
    };
};

} // namespace SlidingGate
