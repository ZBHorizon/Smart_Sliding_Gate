/**
 * @file job.hpp
 * @brief Deklaration der job-Klasse zur Erstellung und Steuerung von Bewegungsabläufen.
 */

#pragma once
#include <list>
#include <limits>
#include <cmath>
namespace SlidingGate {
class job {
public:
    /**
     * @brief Struktur für einen Keyframe.
     */
    struct keyframe {
        float speed;     ///< Geschwindigkeit
        float position;  ///< Position
    };
    /**
     * @brief Erstellt einen neuen Job basierend auf dem Ziel-Keyframe.
     * @param target_keyframe Ziel-Keyframe mit gewünschter Geschwindigkeit und Position.
     * @return true, wenn der Job erfolgreich erstellt wurde.
     */
    static bool create_job(keyframe target_keyframe);

    static void stop_motor();

    /**
     * @brief Löscht den aktuellen Job.
     */
    static void delete_job();

    static bool is_job_active();

    /**
     * @brief Gibt einen interpolierten Geschwindigkeitswert für eine gegebene Position zurück.
     * @param position aktuelle Position
     * @return interpolierte Geschwindigkeit oder signaling_NaN() bei ungültiger Anfrage.
     */
    static float get_speed(float position);


private:
    static std::list<keyframe> keyframes;  ///< Liste der Keyframes

    static constexpr float TOLERANCE = 0.01f;        ///< Positions-Toleranz
    static constexpr float RAMP_DISTANCE = 0.03f;      ///< Anfahr-/Abbremsstrecke
    static constexpr float MINIMAL_SPEED = 0.05f;      ///< Minimalgeschwindigkeit

    static std::list<keyframe>::iterator current_iter; ///< Iterator auf den aktuellen Keyframe
};
}