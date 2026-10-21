/**
 * @file Cinta.hpp
 * @brief Sub-maquina de estados de la cinta transportadora (Motor DC + Sensor IR de fin de cinta).
 * @details Region ortogonal a la FSM global del robot: decide localmente el frenado ante la
 *          deteccion del sensor (sin ida y vuelta por UART) y el rearranque una vez que el
 *          orquestador libera el pick. No conoce el protocolo UART: informa eventos al llamador.
 */

#ifndef CINTA_HPP
#define CINTA_HPP

#include <stdint.h>
#include "MotorDC.hpp"
#include "SensorIR.hpp"

/**
 * @enum EstadoCinta
 * @brief Fase operativa de la cinta transportadora.
 */
enum class EstadoCinta : uint8_t {
    Deshabilitada,  /**< Motor detenido. No reacciona al sensor. Estado seguro por defecto. */
    Avanzando,      /**< Armada y en movimiento a velocidad nominal, esperando una caja. */
    EsperandoPick,  /**< Armada y detenida por deteccion. Espera liberacion del orquestador. */
    Manual          /**< Marcha manual de prueba/calibracion (fuera del ciclo automatico). */
};

/**
 * @enum EventoCinta
 * @brief Eventos que la cinta informa al orquestador.
 */
enum class EventoCinta : uint8_t {
    Ninguno,            /**< Sin novedades. */
    ObjetoDetectado     /**< Caja en posicion de pick (cinta armada y detenida). */
};

class Cinta {
private:
    MotorDC& _motor;                    /**< Etapa de potencia (TIM2_CH3, PB10) */
    SensorIR& _sensor;                  /**< Sensor IR de fin de cinta (PB12) */

    float _velocidadNominal;            /**< Consigna normalizada del ciclo automatico [0.0 a 1.0] */
    uint32_t _rampaArranqueMs;          /**< Duracion de la rampa lineal de arranque */

    EstadoCinta _estado;                /**< Estado activo de la sub-maquina */

    // Generador de rampa (basado en tiempo, independiente de la tasa del loop)
    float _velocidadObjetivo;           /**< Consigna final de la rampa en curso */
    float _velocidadInicioRampa;        /**< Velocidad aplicada al iniciar la rampa */
    float _velocidadAplicada;           /**< Ultima velocidad inyectada al motor */
    uint32_t _inicioRampaMs;            /**< Marca de tiempo de inicio de la rampa */

    bool _pickLiberado;                 /**< Liberacion pendiente: rearrancar cuando el sensor quede libre */

    /**
     * @brief Inicia una rampa lineal desde la velocidad aplicada hasta el objetivo.
     * @note Si el objetivo es menor o igual a la velocidad actual se aplica de inmediato.
     */
    void iniciarRampa(float objetivo, uint32_t nowMs);

    /**
     * @brief Corta la potencia del motor sin rampa (CCR = 0). Frenado por inercia (Coast).
     */
    void detenerInmediato();

    /**
     * @brief Inyecta la velocidad al motor solo si cambio respecto de la ultima aplicada.
     */
    void aplicarVelocidad(float velocidad);

public:
    /**
     * @brief Constructor de la cinta transportadora.
     * @param motor Motor DC ya configurado sobre su canal PWM.
     * @param sensor Sensor IR de fin de cinta.
     * @param velocidadNominal Consigna del ciclo automatico [0.0 a 1.0].
     * @param rampaArranqueMs Duracion de la rampa de arranque en milisegundos.
     */
    Cinta(MotorDC& motor, SensorIR& sensor, float velocidadNominal, uint32_t rampaArranqueMs);

    /**
     * @brief Garantiza el estado de reposo durante el arranque del microcontrolador.
     * @param nowMs Tiempo del sistema (HAL_GetTick()).
     * @note El sensor debe estar inicializado previamente (SensorIR::init).
     */
    void init(uint32_t nowMs);

    /**
     * @brief Evalua sensor, sub-maquina y rampa. Obligatorio invocar en el bucle principal.
     * @param nowMs Tiempo del sistema (HAL_GetTick()).
     * @return ObjetoDetectado en el flanco que detiene la cinta armada.
     * @note Unico punto del firmware que invoca SensorIR::update (consumir el flanco dos veces lo pierde).
     */
    EventoCinta update(uint32_t nowMs);

    /**
     * @brief Habilita el ciclo automatico. Idempotente si ya esta armada.
     * @return ObjetoDetectado si ya habia una caja frente al sensor (la cinta queda detenida).
     */
    EventoCinta armar(uint32_t nowMs);

    /**
     * @brief Detiene inmediatamente la cinta y la deshabilita. Valido desde cualquier estado.
     */
    void desarmar();

    /**
     * @brief Notifica que el robot completo el pick y salio de la zona de la cinta.
     * @note El rearranque efectivo ocurre en update() cuando el sensor este libre.
     */
    void liberarPick();

    /**
     * @brief Descarta una liberacion pendiente (el robot inicia una nueva trayectoria de pick).
     * @note Evita que un pick reintentado arranque la cinta apenas el brazo levanta la caja.
     */
    void cancelarLiberacion();

    /**
     * @brief Marcha manual de prueba/calibracion.
     * @param velocidad Consigna normalizada [0.0 a 1.0]. 0 detiene y deshabilita.
     * @return false si se pidio marcha con una caja frente al sensor (no se mueve).
     */
    bool setVelocidadManual(float velocidad, uint32_t nowMs);

    /**
     * @brief Retorna el estado actual de la cinta.
     */
    EstadoCinta getEstado() const;

    /**
     * @brief Retorna true si el sensor (ya filtrado) detecta un objeto.
     */
    bool objetoPresente() const;

    /**
     * @brief Nombre del estado para el protocolo UART (EVT:CINTA,<NOMBRE>).
     */
    static const char* toString(EstadoCinta estado);
};

#endif /* CINTA_HPP */
