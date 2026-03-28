/**
 * @file Joystick.hpp
 * @brief Definicion de la clase Joystick para la lectura analogica y digital.
 * @note Diseñado para microcontroladores STM32 utilizando DMA y EXTI
 */

#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <stdint.h>
#include "stm32f1xx_hal.h"

class Joystick {
private:
	// Punteros a memoria RAM vinculados por DMA
	volatile uint32_t* ptr_raw_x;		/**< Puntero al registro en RAM actualizado por DMA para el Eje X */
	volatile uint32_t* ptr_raw_y;		/**< Puntero al registro en RAM actualizado por DMA para el Eje Y */

	GPIO_TypeDef* puerto_sw;			/**< Puerto GPIO asignado al boton fisico*/
	uint16_t pin_sw;					/**< Pin GPIO asignado al boton fisico*/

	float valor_x;						/**< Valor espacial normalizado actual del eje X [-1.0 1.0] */
	float valor_y;						/**< Valor espacial normalizado actual del eje Y [-1.0 1.0] */
	uint32_t raw_x;						/**< Última lectura cruda del ADC para el Eje X [0 a 4095] */
	uint32_t raw_y;						/**< Última lectura cruda del ADC para el Eje Y [0 a 4095] */

	float centro_x;						/**< Lectura cruda del ADC cuando el Eje X está en reposo */
	float centro_y;						/**< Lectura cruda del ADC cuando el Eje Y está en reposo */
	float zona_muerta;					/**< Umbral de tolerancia alrededor del centro [0.0 a 1.0] */

	// Control Antirrebote
	volatile uint32_t tiempo_inicio_debounce;	/**< Marca de tiempo (ms) en que se detectó la interrupción inicial */
	volatile bool en_proceso_debounce;			/**< Bandera que indica si existe una evaluación de pulsación en curso */


	// Constantes de operacion
	static constexpr float ADC_MAX_VALOR = 4095.0f;
	static constexpr uint32_t RETARDO_ANTIRREBOTE_MS = 50;

	// Filtro EMA y Discretizacion
	float _alpha;						/**< Valor de filtro (entre 0.0 y 1.0) */
	float _estado_filtrado_x;
	float _estado_filtrado_y;
	float _niveles_resolucion;

	// Constantes Pre-calculadas (inversos multiplicativos para evitar divisiones en punto flotante)
	float _k_pos_x;
	float _k_neg_x;
	float _k_pos_y;
	float _k_neg_y;
	float _k_deadzone;
	float _k_EMA_comp;
	float _k_niveles;

	/**
	 * @brief Recalcula los inversos multiplicativos internos.
	 * @note Invocado en la construccion y ante cambios de parametros dinamicos.
	 */
	void CalcularMultiplicadores();

	/**
	 * @brief Ejecuta el pipeline DSP sobre la lectura en crudo mediante multiplicaciones.
	 * @param valor_adc Lectura en bruto del ADC.
	 * @param centro Centro calibrado del eje.
	 * @param k_pos Inverso multiplicativo de la pendiente positiva.
	 * @param k_neg Inverso multiplicativo de la pendiente negativa.
	 * @param estado_previo Referencia al estado histórico del filtro EMA.
	 * @return Valor espacial procesado [-1.0f, 1.0f].
	 */
	float ProcesarLectura(uint32_t valor_adc, float centro, float k_pos, float k_neg, float& estado_previo);

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
	 * @brief Lee los datos de la memoria DMA y actualiza los cálculos espaciales.
	 * @note Esta función debe ser llamada periódicamente desde el bucle principal.
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
     * @return true si la pulsación es legítima y estable, false si está rebotando o inactivo.
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
	 * @brief Define la cantidad de niveles discretos de velocidad.
	 * @param niveles Ej. 10.0f creará saltos de 0.1, 0.2, 0.3...
	 */
	 void setNivelesResolucion(float niveles);
};

#endif /* JOYSTICK_HPP */
