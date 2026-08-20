/**
 * @file JoystickInput.hpp
 * @brief Acceso a hardware crudo del joystick: lecturas ADC via DMA y boton fisico con antirrebote.
 * @note Sin logica de calibracion/DSP. Ver JoystickFilter para el procesamiento de senal.
 * @note Incluye un filtro decimador tipo "integrate-and-dump": el ADC/DMA refresca el buffer
 * compartido a ~11.9kHz de forma continua, muy por encima de los 100Hz a los que el joystick
 * se muestrea realmente (ver Config::JOYSTICK_SAMPLE_PERIOD_MS). Sin promediar las muestras
 * intermedias, tomar una unica lectura instantanea cada 10ms deja pasar sin atenuar cualquier
 * ruido de banda ancha por encima de 50Hz (Nyquist de esa decimacion) directo al pipeline de
 * DSP -- el filtro EMA de JoystickFilter, que corre recien despues, no puede deshacer un
 * aliasing que ya ocurrio en el muestreo. AcumularMuestra() debe llamarse en CADA vuelta del
 * bucle principal (sin gate de tasa) para que este promediado sea efectivo.
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

	// Filtro decimador (integrate-and-dump): acumula crudo cada vuelta del loop, se promedia
	// y se reinicia una vez por ciclo de muestreo (100Hz). Ver AcumularMuestra().
	uint32_t _acumuladorX;
	uint32_t _acumuladorY;
	uint32_t _muestrasAcumuladas;

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
	 * @brief Acumula una muestra cruda de ambos ejes (filtro decimador).
	 * @note Debe llamarse SIN limitar la tasa, en cada vuelta del bucle principal -- es lo que
	 * promedia el ruido de banda ancha entre los ~11.9kHz del ADC/DMA y los 100Hz de muestreo
	 * real del joystick, antes de que ese muestreo pueda hacerle aliasing.
	 */
	void AcumularMuestra();

	/**
	 * @brief Promedia las muestras acumuladas desde la ultima llamada y reinicia la ventana de
	 * acumulacion para el proximo ciclo. Debe llamarse una unica vez por ciclo de muestreo
	 * (100Hz, ver Joystick::Actualizar()).
	 * @param outX Salida: promedio del eje X [0 a 4095].
	 * @param outY Salida: promedio del eje Y [0 a 4095].
	 */
	void LeerYReiniciarPromedio(uint32_t& outX, uint32_t& outY);

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
