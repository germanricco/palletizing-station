/**
 * @file Servo.hpp
 * @brief Declaración de la clase Servo para control cinemático de actuadores PWM en STM32.
 * @details Optimizado para sistemas embebidos de tiempo real sin dependencias de librerías pesadas.
 */

#ifndef SERVO_HPP
#define SERVO_HPP

#include <stdint.h>
#include <cmath>


class Servo {
private:
    volatile uint32_t* _ccr_reg;			/**< Puntero de hardware al Registro de Captura y Comparación */

    uint32_t _minPulse;						/**< Ancho de pulso minimo tolerado por el servo */
    uint32_t _maxPulse;						/**< Ancho de pulso maximo tolerado por el servo */
    uint32_t _homePulse;					/**< Pulso correspondiente a la posicion de calibracion inicial */

    float _minPulseFloat;
    float _maxPulseFloat;

    float _maxSpeed;						/**< Limite superior de incremento por ciclo */
    float _stepSize;						/**< Magnitud y direccion del vector de velocidad */
	float _currentCCR;						/**< Representacion continua de coma flotante de la posicion actual */

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
     * @param velocityFactor Factor de velocidad en el rango [-1.0, 1.0].
     */
    void setVelocityCommand(float velocityFactor);

    /**
     * @brief Integra la velocidad para actualizar la posición física.
     * @note PRECAUCIÓN: Esta función debe ejecutarse estrictamente desde una Rutina de Servicio
     * de Interrupción (ISR) con una base de tiempo constante (ej. 50 Hz / 20 ms).
     */
    void updatePosition();

    /**
     * @brief Modifica el límite máximo de velocidad.
     * @param newMaxSpeed Nuevo límite estrictamente positivo.
     */
    void setMaxSpeed(float newMaxSpeed);

    /**
     * @brief Computa la posición angular actual basándose en interpolación lineal.
     * @return float Ángulo en grados [0.0, 180.0].
     */
    float getAngleDegrees() const;

    /**
     * @brief Computa la posición angular actual optimizada para modelos de transformación.
     * @return float Ángulo en radianes [0.0, PI].
     */
    float getAngleRadians() const;

};

#endif /* SERVO_HPP */
