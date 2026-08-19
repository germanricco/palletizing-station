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
    
    // Constantes de Mapeo Físico
    float _minAngle;
    float _angleToPulseFactor;

    // Limites de Seguridad
    int32_t _minPulse_Q16;					/**< Ancho de pulso minimo tolerado por el servo */
    int32_t _maxPulse_Q16;					/**< Ancho de pulso maximo tolerado por el servo */
    int32_t _homePulse_Q16;					/**< Pulso correspondiente a la posicion de calibracion inicial */
    int32_t _maxSpeed_Q16;					/**< Magnitud maxima de incremento por pulso por ciclo */
    int32_t _acceleration_Q16;				/**< Tasa maxima de aceleracion/desaceleracion */
    int32_t _accelDivisor;					/**< Divisor calibrado usado para derivar _acceleration_Q16 desde _maxSpeed_Q16 */

    // Variables de Estado Dinamico
    // NOTA: los siguientes campos se escriben desde contexto main-loop (setters) y se leen/escriben
    // desde la ISR de TIM3 (update()). El acceso a enteros de 32 bits alineados es atomico en
    // Cortex-M3, pero se marcan 'volatile' para evitar que el compilador cachee/reordene estos
    // valores a traves de la frontera main/ISR (evita UB de carrera de datos formal en C++).
	volatile int32_t _currentCCR_Q16;			/**< Representacion continua de coma flotante de la posicion actual */
    volatile int32_t _targetVelocity_Q16;		/**< Velocidad dictada */
    volatile int32_t _currentVelocity_Q16;		/**< Velocidad filtrada/suavizada a inyectar al hardware */
    volatile int32_t _targetPulse_Q16;			/**< Posicion objetiva requerida (utilizada en modo Position) */

    volatile ServoControlMode _controlMode;	/**< Maquina de estados para bifurcacion del control */

    // Constantes matemáticas internas
    static constexpr float Q16_FACTOR = 65536.0f;
    static constexpr uint32_t Q16_HALF_STEP = 32768U; 
    static constexpr int32_t DEADBAND_Q16 = 262144; // 4.0 us (Realidad del MG996R)
    static constexpr int32_t MIN_PHYSICAL_VELOCITY_Q16 = 131072; // 2.0 us/ciclo
    static constexpr int32_t POSITION_GAIN_DIVISOR = 16; // Ganancia proporcional (error/N) de aproximacion final en SmoothPosition
    
    /**
     * @brief Método interno para convertir grados físicos a pulsos Q16
     */
    int32_t convertAngleToPulseQ16(float angleDegrees) const;

public:
    /**
     * @brief Constructor del controlador cinemático.
     * @param ccr_register Puntero al registro CCR del Timer (ej. &TIM2->CCR1).
     * @param minAngle Ángulo físico mínimo.
     * @param maxAngle Ángulo físico máximo.
     * @param minPulseUs Ancho de pulso en microsegundos correspondiente al ángulo mínimo.
     * @param maxPulseUs Ancho de pulso en microsegundos correspondiente al ángulo máximo.
     * @param homeAngle Posición de reposo de la articulación.
     * @param maxSpeed Incremento máximo de posición por actualización.
     * @param accelDivisor Divisor de aceleracion para diferencias en la inercia
     */
    Servo(volatile uint32_t* ccr_register,
    		float minAngle, float maxAngle,
            float minPulseUs, float maxPulseUs,
			float homeAngle,
			float maxSpeed, int32_t accelDivisor = 10);

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
    void setTargetVelocity(float normalizedVelocity);

    /**
     * @brief Establece un comando de posicionamiento absoluto
     * @param angleDegrees Angulo fisico requerido en el espacio de trabajo
     * @note Transfiere el actuado a ServoControlMode::Position
     */
    void setSmoothTargetPosition(float angleDegrees);

    /**
     * @brief Establece un comando para posicionamiento dictado por trayectoria
     * %param angleDegrees Angulo fisico requerido
     */
    void setDirectTargetPosition(float angleDegrees);

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
