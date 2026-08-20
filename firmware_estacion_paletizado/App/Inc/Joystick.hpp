/**
 * @file Joystick.hpp
 * @brief Fachada del joystick: compone JoystickInput (hardware) y JoystickFilter (DSP por eje).
 * @note Diseñado para microcontroladores STM32 utilizando DMA y EXTI.
 * @note Actualizar() debe llamarse a tasa fija (ver Config::JOYSTICK_SAMPLE_PERIOD_MS) para que
 * el filtro EMA de cada eje tenga una frecuencia de corte determinista. AcumularMuestra(), en
 * cambio, debe llamarse en CADA vuelta del bucle principal sin gate de tasa -- es el filtro
 * decimador que promedia el ADC (~11.9kHz) antes de que Actualizar() lo muestree a 100Hz.
 */

#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "Config.hpp"
#include "JoystickInput.hpp"
#include "JoystickFilter.hpp"

class Joystick {
private:
	JoystickInput _input;
	JoystickFilter _filtroX;
	JoystickFilter _filtroY;

	uint32_t raw_x;						/**< Ultima lectura cruda del ADC para el Eje X [0 a 4095] */
	uint32_t raw_y;						/**< Ultima lectura cruda del ADC para el Eje Y [0 a 4095] */

public:
	/**
	 * @brief Constructor del objeto Joystick.
	 * @param direccion_memoria_x Puntero a la posición del arreglo DMA para el Eje X.
	 * @param direccion_memoria_y Puntero a la posición del arreglo DMA para el Eje Y.
	 * @param puerto_sw_asignado Puerto GPIO del microcontrolador conectado al pin SW.
	 * @param pin_sw_asignado Número de pin conectado al botón SW.
	 * @param calibracion_centro_x Valor de reposo del Eje X (Por defecto: 2048.0).
	 * @param calibracion_centro_y Valor de reposo del Eje Y (Por defecto: 2048.0).
	 * @param umbral_zona_muerta Porcentaje del recorrido ignorado alrededor del centro (Por defecto: 0.05).
	 */
	Joystick(volatile uint32_t* direccion_memoria_x, volatile uint32_t* direccion_memoria_y,
			GPIO_TypeDef* puerto_sw_asignado, uint16_t pin_sw_asignado,
			float calibracion_centro_x = 2048.0f, float calibracion_centro_y = 2048.0f,
			float umbral_zona_muerta = 0.04f);

	/**
	 * @brief Acumula una muestra cruda del ADC para el filtro decimador.
	 * @note Debe llamarse en CADA vuelta del bucle principal, sin gate de tasa (ver nota de clase).
	 */
	void AcumularMuestra();

	/**
	 * @brief Promedia las muestras acumuladas desde la última llamada y actualiza los cálculos
	 * espaciales a partir de ese promedio.
	 * @note Esta función debe ser llamada periódicamente a tasa fija (ver nota de clase),
	 * no en cada vuelta del bucle principal.
	 */
    void Actualizar();

    /**
     * @brief Inicia el temporizador de la FSM de antirrebote.
     * @param pin_disparado El identificador del pin que generó la interrupción EXTI.
     * @note Esta función no bloquea el sistema. Debe llamarse al detectar la interrupción.
     */
    void NotificarInterrupcion(uint16_t pin_disparado);

    /**
     * @brief Evalúa el estado del botón tras expirar la ventana de tiempo.
     * @return true si la pulsación es legítima y estable, false si está rebotando o inactiva.
     * @note Esta función debe sondearse continuamente en el bucle principal.
     */
    bool HayPulsacionValida();

    /* Funciones de acceso (Getters) */
    float ObtenerX() const;
    float ObtenerY() const;
    uint32_t ObtenerRawX() const;
    uint32_t ObtenerRawY() const;

	/**
	 * @brief Modifica el factor de suavizado del filtro EMA dinamicamente
	 * @param alpha Valores entre 0.0 (max suavizado) y 1.0 (sin filtro)
	 */
	void setFiltroAlpha(float alpha);


	/**
	 * @brief Modifica la curvatura de la respuesta del joystick
	 */
	void setCurvatura(float curvatura);
};

#endif /* JOYSTICK_HPP */
