/**
 * @file Actuador.hpp
 * @brief Clase para el control neumático de efectores finales (Bomba de vacío y Válvula).
 * @details Implementa una máquina de estados asíncrona mediante PWM para control de potencia.
 */

#ifndef ACTUADOR_HPP
#define ACTUADOR_HPP

#include <stdint.h>

/**
 * @enum ActuatorState
 * @brief Define la fase operativa del ciclo neumático.
 */
enum class ActuatorState : uint8_t {
    Idle,       /**< Reposo absoluto. Energía desconectada. */
    Suction,    /**< Generación de vacío activo. Bomba ON, Válvula CERRADA. */
    Holding,    /**< Mantenimiento de vacío. Bomba OFF/Bajo PWM, Válvula CERRADA. */
    Releasing   /**< Ruptura de vacío (Purga). Bomba OFF, Válvula ABIERTA. */
};

class Actuador {
private:
    volatile uint32_t* _ccr_pump;        /**< Registro CCR del Timer para la Bomba */
    volatile uint32_t* _ccr_valve;       /**< Registro CCR del Timer para la Válvula */
    uint32_t _arr_max;                   /**< Resolución máxima del Timer (100% Duty Cycle) */

    ActuatorState _currentState;         /**< Estado activo de la máquina de estados */

    // Variables para la temporización no bloqueante
    uint32_t _release_start_time_ms;     /**< Marca de tiempo al iniciar la purga */
    uint32_t _release_duration_ms;       /**< Tiempo paramétrico necesario para soltar la pieza */

    /**
     * @brief Inyecta asíncronamente el ciclo de trabajo al registro del hardware.
     */
    void setPumpDutyCycle(float normalized_duty);
    void setValveDutyCycle(float normalized_duty);

public:
    /**
     * @brief Constructor del controlador neumático.
     * @param ccr_pump Puntero al registro PWM de la bomba.
     * @param ccr_valve Puntero al registro PWM de la electroválvula.
     * @param arr_max Valor de autorrecarga del temporizador (Definición del 100%).
     * @param release_duration_ms Milisegundos que la válvula permanecerá abierta al purgar.
     */
    Actuador(volatile uint32_t* ccr_pump, volatile uint32_t* ccr_valve,
             uint32_t arr_max, uint32_t release_duration_ms = 800);

    /**
     * @brief Asegura el estado de reposo durante el arranque del microcontrolador.
     */
    void init();

    /**
     * @brief Inicia la generación de vacío.
     * @param power Potencia de la bomba [0.0f a 1.0f]. Permite vacío parcial para piezas frágiles.
     */
    void commandSuction(float power = 1.0f);

    /**
     * @brief Detiene la succión conservando el vacío residual en el circuito neumático.
     * @param maintenance_power Potencia residual opcional para mitigar fugas.
     */
    void commandHold(float maintenance_power = 0.0f);

    /**
     * @brief Abre la válvula para ecualizar la presión atmosférica y soltar la pieza.
     * @note Transfiere el control temporal a la función update().
     */
    void commandRelease();

    /**
     * @brief Fuerza el paso inmediato al estado de reposo absoluto.
     */
    void commandIdle();

    /**
     * @brief Evalúa la máquina de estados. Obligatorio invocar en el bucle principal.
     * @param current_time_ms Tiempo del sistema (HAL_GetTick()).
     */
    void update(uint32_t current_time_ms);

    /**
     * @brief Retorna el estado actual del efector final.
     */
    ActuatorState getState() const;
};

#endif /* ACTUADOR_HPP */
