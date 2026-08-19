/**
 * @file JoystickInput.hpp
 * @brief Acceso a hardware crudo del joystick: lecturas ADC via DMA y boton fisico con antirrebote.
 * @note Sin logica de calibracion/DSP. Ver JoystickFilter para el procesamiento de senal.
 */

#ifndef JOYSTICK_INPUT_HPP
#define JOYSTICK_INPUT_HPP

#include <stdint.h>
#include "stm32f1xx_hal.h"

class JoystickInput {
private:
	// Punteros a memoria RAM vinculados por DMA
	volatile uint32_t* ptr_raw_x;		/**< Puntero al registro en RAM actualizado por DMA para el Eje X */
	volatile uint32_t* ptr_raw_y;		/**< Puntero al registro en RAM actualizado por DMA para el Eje Y */

	GPIO_TypeDef* puerto_sw;			/**< Puerto GPIO asignado al boton fisico*/
	uint16_t pin_sw;					/**< Pin GPIO asignado al boton fisico*/

	// Control Antirrebote
	volatile uint32_t tiempo_inicio_debounce;	/**< Marca de tiempo (ms) en que se detecto la interrupcion inicial */
	volatile bool en_proceso_debounce;			/**< Bandera que indica si existe una evaluacion de pulsacion en curso */

	static constexpr uint32_t RETARDO_ANTIRREBOTE_MS = 50;

public:
	/**
	 * @brief Constructor del acceso a hardware del joystick.
	 * @param direccion_memoria_x Puntero a la posicion del arreglo DMA para el Eje X.
	 * @param direccion_memoria_y Puntero a la posicion del arreglo DMA para el Eje Y.
	 * @param puerto_sw_asignado Puerto GPIO del microcontrolador conectado al pin SW.
	 * @param pin_sw_asignado Numero de pin conectado al boton SW.
	 */
	JoystickInput(volatile uint32_t* direccion_memoria_x, volatile uint32_t* direccion_memoria_y,
			GPIO_TypeDef* puerto_sw_asignado, uint16_t pin_sw_asignado);

	/**
	 * @brief Lectura cruda actual del ADC para el Eje X [0 a 4095].
	 */
	uint32_t LeerRawX() const;

	/**
	 * @brief Lectura cruda actual del ADC para el Eje Y [0 a 4095].
	 */
	uint32_t LeerRawY() const;

	/**
	 * @brief Inicia el temporizador de la FSM de antirrebote.
	 * @param pin_disparado El identificador del pin que genero la interrupcion EXTI.
	 * @note Esta funcion no bloquea el sistema. Debe llamarse al detectar la interrupcion.
	 */
	void NotificarInterrupcion(uint16_t pin_disparado);

	/**
	 * @brief Evalua el estado del boton tras expirar la ventana de tiempo.
	 * @return true si la pulsacion es legitima y estable, false si esta rebotando o inactiva.
	 * @note Esta funcion no bloquea el sistema. Debe sondearse continuamente en el bucle principal.
	 */
	bool HayPulsacionValida();
};

#endif /* JOYSTICK_INPUT_HPP */
