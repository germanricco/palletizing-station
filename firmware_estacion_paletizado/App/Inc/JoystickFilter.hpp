/**
 * @file JoystickFilter.hpp
 * @brief Pipeline DSP puro (sin hardware) para acondicionar un eje analogico de joystick.
 * @note Recibe lecturas crudas por parametro; no conoce DMA/ADC. Debe invocarse a una tasa fija
 * (ver Config::JOYSTICK_SAMPLE_PERIOD_MS) para que el filtro EMA tenga una frecuencia de corte
 * determinista.
 */

#ifndef JOYSTICK_FILTER_HPP
#define JOYSTICK_FILTER_HPP

#include <stdint.h>
#include "Config.hpp"

class JoystickFilter {
private:
	float centro;						/**< Lectura cruda del ADC cuando el eje esta en reposo */
	float zona_muerta;					/**< Umbral de tolerancia alrededor del centro [0.0 a 1.0] */

	// Filtro EMA y Discretizacion
	float _alpha;						/**< Valor de filtro (entre 0.0 y 1.0) */
	float _estado_filtrado;

	// Constantes Pre-calculadas (inversos multiplicativos para evitar divisiones en punto flotante)
	float _k_pos;
	float _k_neg;
	float _k_deadzone;
	float _k_EMA_comp;

	float _curvatura;

	static constexpr float ADC_MAX_VALOR = Config::ADC_MAX_RESOLUTION;

	/**
	 * @brief Recalcula los inversos multiplicativos internos.
	 * @note Invocado en la construccion y ante cambios de parametros dinamicos.
	 */
	void CalcularMultiplicadores();

public:
	/**
	 * @brief Constructor del filtro DSP de un eje.
	 * @param calibracion_centro Valor de reposo del eje (lectura ADC cruda).
	 * @param umbral_zona_muerta Porcentaje del recorrido ignorado alrededor del centro.
	 * @param alpha Factor de suavizado EMA, calibrado para Config::JOYSTICK_SAMPLE_PERIOD_MS.
	 * @param curvatura Factor de mezcla lineal/cubica de la respuesta.
	 */
	explicit JoystickFilter(float calibracion_centro,
			float umbral_zona_muerta = Config::JOYSTICK_DEADZONE,
			float alpha = Config::JOYSTICK_EMA_ALPHA,
			float curvatura = Config::JOYSTICK_CURVATURE);

	/**
	 * @brief Ejecuta el pipeline DSP sobre una lectura en crudo mediante multiplicaciones.
	 * @param valor_adc Lectura en bruto del ADC.
	 * @return Valor espacial procesado [-1.0f, 1.0f].
	 * @note Debe llamarse a una tasa fija (ver nota de clase) para mantener el corte del EMA.
	 */
	float Procesar(uint32_t valor_adc);

	/**
	 * @brief Ultimo valor procesado (sin volver a muestrear).
	 */
	float Valor() const;

	/**
	 * @brief Modifica el factor de suavizado del filtro EMA dinamicamente.
	 * @param alpha Valores entre 0.0 (max suavizado) y 1.0 (sin filtro).
	 */
	void setAlpha(float alpha);

	/**
	 * @brief Modifica la curvatura de la respuesta.
	 */
	void setCurvatura(float curvatura);
};

#endif /* JOYSTICK_FILTER_HPP */
