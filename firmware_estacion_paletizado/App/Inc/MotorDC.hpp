/**
 * @file MotorDC.hpp
 * @brief Definicion de la clase MotorDC para el control de actuadores DC mediante PWM
 * @note Compatible con puentes H (bidireccional) y topologias MOSFET simples (unidireccional)
 */

#ifndef MOTORDC_HPP
#define MOTORDC_HPP

#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @enum MotorBrakeMode
 * @brief Define la topologia de disipacion de energia
 */
enum class MotorBrakeMode: uint8_t {
	Coast,	/**< Alta impedancia. El motor gira libremente por inercia */
	Brake	/**< Cortocircuito a GND o VCC. Frenado Electromagnetico*/
};

class MotorDC {
private:
	// Enlace de hardware de potencia (Obligatorio)
	volatile uint32_t* _ccr_reg;	/**< Puntero a registro CCR del timer asignado para PWM */
	uint32_t _arr_max_value;		/**< Maximo valor del registro ARR del Timer asignado */
	uint32_t _min_ccr_value;		/**< Umbral minimo para vencer friccion estatica */
	uint32_t _dynamic_ccr_range;	/**< Rango util precalculado (_arr_max_value - _min_duty_cycle) */

	int32_t _input_deadband_Q16;	/**< Filtro parametrico de ruido para la intencion de entrada (Q16.16) */

	// Enlaces de hardware direccional (para puentes H)
	GPIO_TypeDef* _port_in1;
	uint32_t _pin_in1_set;			/**< Mascara BSRR precalculada para SET */
	uint32_t _pin_in1_reset;		/**< Mascara BSRR precalculada para RESET */

	GPIO_TypeDef* _port_in2;
	uint32_t _pin_in2_set;
	uint32_t _pin_in2_reset;

	// Estado interno
	int32_t _currentSpeed_Q16;		/**< Velocidad instantanea en formato Q16.16 */
	bool _is_bidirectional;			/**< Bandera de topologia inferida en construccion */
	MotorBrakeMode _brake_mode;		/**< Comportamiento de detencion configurado */

public:
	/**
	 * @brief Constructor del Motor DC.
	 * @param ccr_register Puntero directo al registro CCR.
	 * @param arr_max Valor máximo del registro de autorrecarga (ej. 999).
	 * @param min_ccr Valor minimo en ticks de CCR para arrancar el motor.
	 * @param input_deadband Fracción [0.0 a 1.0] para ignorar ruido en velocidad = 0 (Entrada).
	 * @param brake_mode Comportamiento al detenerse (Coast o Brake)
	 * @param port_in1 Puerto GPIO para IN1 (Omitir para unidireccional).
	 * @param pin_in1 Pin GPIO para IN1.
	 * @param port_in2 Puerto GPIO para IN2 (Omitir para unidireccional).
	 * @param pin_in2 Pin GPIO para IN2.
	 */
	MotorDC(volatile uint32_t* ccr_register, uint32_t arr_max, uint32_t min_ccr,
			float input_deadband = 0.001f,
			MotorBrakeMode brake_mode = MotorBrakeMode::Coast,
			GPIO_TypeDef* port_in1 = nullptr, uint16_t pin_in1 = 0,
	        GPIO_TypeDef* port_in2 = nullptr, uint16_t pin_in2 = 0);

	/**
	 * @brief Inicializa el motor garantizando un estado de reposo absoluto.
	 */
	void init();

	/**
	 * @brief Establece la velocidad y direccion operativa del motor
	 * @param normalized_speed Magnitud direccional en el rango [-1.0, 1.0]
	 * @note No requiere llamadas periodicas ni interrupciones. El hardware mantiene el estado.
	 */
	void setSpeed(float normalized_speed);

	/**
	 * @brief Sobrecarga de alto rendimiento para evitar conversiones flotantes en cascada.
	 * @param speed_Q16 Velocidad normalizada pre-escalada en formado Q16.16.
	 */
	void setSpeedQ16(int32_t speed_Q16);
};

#endif /* MOTORDC_HPP */
