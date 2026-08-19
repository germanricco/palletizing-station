/**
 * @file JoystickFilter.cpp
 * @brief Implementacion del pipeline DSP puro de un eje de joystick.
 */

#include "JoystickFilter.hpp"

JoystickFilter::JoystickFilter(float calibracion_centro, float umbral_zona_muerta,
		float alpha, float curvatura)
    : centro(calibracion_centro),
	  zona_muerta(umbral_zona_muerta),
	  _alpha(alpha),
	  _estado_filtrado(0.0f),
	  _curvatura(curvatura) {

	// Ejecucion del pre-calculo para evitar divisiones en tiempo de ejecucion
	CalcularMultiplicadores();
}

void JoystickFilter::CalcularMultiplicadores() {
    _k_pos = 1.0f / (ADC_MAX_VALOR - centro);
    _k_neg = 1.0f / centro;

    _k_deadzone = 1.0f / (1.0f - zona_muerta);
    _k_EMA_comp = 1.0f - _alpha;
}

float JoystickFilter::Procesar(uint32_t valor_adc) {
    float valor_flotante = static_cast<float>(valor_adc);

	// 1. Normalizacion asimetrica
	float N = (valor_flotante >= centro) ?
			(valor_flotante - centro) * _k_pos :
			(valor_flotante - centro) * _k_neg;

    // 2. Saturacion (Hardware limites bounding)
	N = (N > 1.0f) ? 1.0f : ((N < -1.0f) ? -1.0f : N);

	// 3. Compensacion de Zona Muerta lineal
    float target = 0.0f;
    if (N > zona_muerta) {
    	target = (N - zona_muerta) * _k_deadzone;
    } else if (N < -zona_muerta) {
    	target = (N + zona_muerta) * _k_deadzone;
    }

    // 4. Mapeo No Lineal (Interpolar entre respuesta lineal y cubica)
    if (target != 0.0f) {
        float cubo = target * target * target;
        target = (1.0f - _curvatura) * target + _curvatura * cubo;
    }

    // 5. Filtro EMA (Ts fijo, ver Config::JOYSTICK_SAMPLE_PERIOD_MS)
    _estado_filtrado = (_alpha * target) + (_k_EMA_comp * _estado_filtrado);

    // Retorno continuo al dominio [-1.0f, 1.0f]
    return _estado_filtrado;
}

float JoystickFilter::Valor() const { return _estado_filtrado; }

void JoystickFilter::setAlpha(float alpha) {
    _alpha = (alpha < 0.01f) ? 0.01f : ((alpha > 1.0f) ? 1.0f : alpha);
    _k_EMA_comp = 1.0f - _alpha;
}

void JoystickFilter::setCurvatura(float curvatura) {
    _curvatura = (curvatura < 0.0f) ? 0.0f : ((curvatura > 1.0f) ? 1.0f : curvatura);
}
