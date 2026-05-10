/**
 * @file Servo.hpp
 * @brief Declaración de la clase Servo para control cinemático dual (Velocidad/Posicion)
 * @details Optimizado para sistemas embebidos de tiempo real..
 */

#ifndef SERVO_HPP
#define SERVO_HPP

#include <stdint.h>
#include <cmath>

/**
 * @enum ServoControlMode
 * @brief Define la topologia de control activo para la articulacion.
 */
enum class ServoControlMode : uint8_t {
	Velocity = 0,		/**< Integracion continua gobernada por dispositivo de entrada */
	SmoothPosition = 1,		/**< Busqueda de un objetivo angular especifico */
	DirectPosition = 2
};

class Servo {
private:
    volatile uint32_t* _ccr_reg;			/**< Puntero de hardware al Registro de Captura y Comparación */

    // Limites de Seguridad
    int32_t _minPulse_Q16;					/**< Ancho de pulso minimo tolerado por el servo */
    int32_t _maxPulse_Q16;					/**< Ancho de pulso maximo tolerado por el servo */
    int32_t _homePulse_Q16;					/**< Pulso correspondiente a la posicion de calibracion inicial */
    int32_t _maxSpeed_Q16;					/**< Magnitud maxima de incremento por pulso por ciclo */
    int32_t _acceleration_Q16;				/**< Tasa maxima de aceleracion/desaceleracion */

    // Variables de Estado Dinamico
	int32_t _currentCCR_Q16;				/**< Representacion continua de coma flotante de la posicion actual */
    int32_t _targetVelocity_Q16;			/**< Velocidad dictada */
    int32_t _currentVelocity_Q16;			/**< Velocidad filtrada/suavizada a inyectar al hardware */
    int32_t _targetPulse_Q16;				/**< Posicion objetiva requerida (utilizada en modo Position) */

    ServoControlMode _controlMode;			/**< Maquina de estados para bifurcacion del control */

public:
    /**
     * @brief Constructor del controlador de servomotor.
     * @param ccr_register Puntero al registro CCR  del Timer.
     * @param minP Límite mecánico inferior en ticks de temporizador.
     * @param maxP Límite mecánico superior en ticks de temporizador.
     * @param homeP Posición de reposo por defecto de la articulación.
     * @param minSpeed Umbral de velocidad para evitar zonas muertas del actuador.
     * @param maxSpeed Incremento máximo de posición por actualización.
     */
    Servo(volatile uint32_t* ccr_register,
    		float minAngle,
			float maxAngle,
			float homeAngle,
			float maxSpeed);

    /**
     * @brief Inicializa el sistema y posiciona el actuador de forma segura.
     * @note Debe ser invocado una única vez en la fase de inicialización antes de arrancar el PWM.
     */
    void init();

    /**
     * @brief Retorna mecanicamente la articulacion a su posicion de calibracion base.
     */
    void goHome();

    /**
     * @brief Establece el comando de velocidad para la articulación.
     * @param normalizedVelocity Factor de velocidad en el rango [-1.0, 1.0].
     * @note Transfiere el actuador a ServoControlMode::Velocity
     */
    void commandVelocity(float normalizedVelocity);

    /**
     * @brief Establece un comando de posicionamiento absoluto
     * @param angleDegrees Angulo fisico requerido en el espacio de trabajo
     * @note Transfiere el actuado a ServoControlMode::Position
     */
    void commandSmoothPosition(float angleDegrees);

    /**
     * @brief Establece un comando para posicionamiento dictado por trayectoria
     * %param angleDegrees Angulo fisico requerido
     */
    void commandDirectPosition(float angleDegrees);

    /**
     * @brief Integra la velocidad para actualizar la posición física.
     * @note PRECAUCIÓN: Esta función debe ejecutarse estrictamente desde una Rutina de Servicio
     * de Interrupción (ISR) con una base de tiempo constante (ej. 50 Hz / 20 ms).
     */
    void update();

    /**
     * @brief Modifica el límite máximo de velocidad.
     * @param newMaxSpeed Nuevo límite estrictamente positivo.
     */
    void setMaxSpeed(float newMaxSpeed);

    /* Getters Cinematicos */
    float getAngleDegrees() const;
    float getAngleRadians() const;

    bool isTargetReached() const;

};

#endif /* SERVO_HPP */
