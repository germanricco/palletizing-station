/**
 * @file Joystick.cpp
 * @brief Implementación de la clase Joystick.
 */

#include "Joystick.hpp"


Joystick::Joystick(volatile uint32_t* direccion_memoria_x, volatile uint32_t* direccion_memoria_y,
		GPIO_TypeDef* puerto_sw_asignado, uint16_t pin_sw_asignado,
		float calibracion_centro_x, float calibracion_centro_y, float umbral_zona_muerta)
    : ptr_raw_x(direccion_memoria_x), ptr_raw_y(direccion_memoria_y),
	  puerto_sw(puerto_sw_asignado), pin_sw(pin_sw_asignado),
	  valor_x(0.0f), valor_y(0.0f), raw_x(0), raw_y(0),
	  centro_x(calibracion_centro_x), centro_y(calibracion_centro_y),
	  zona_muerta(Config::JOYSTICK_DEADZONE),
	  tiempo_inicio_debounce(0), en_proceso_debounce(false),
	  _alpha(Config::JOYSTICK_EMA_ALPHA),
	  _estado_filtrado_x(0.0f), _estado_filtrado_y(0.0f),
	  _curvatura(Config::JOYSTICK_CURVATURE) {

	// Ejecucion del pre-calculo para evitar divisiones en tiempo de ejecucion
	CalcularMultiplicadores();
}

void Joystick::CalcularMultiplicadores() {
    // Eje X
    _k_pos_x = 1.0f / (ADC_MAX_VALOR - centro_x);
    _k_neg_x = 1.0f / centro_x;

    // Eje Y
    _k_pos_y = 1.0f / (ADC_MAX_VALOR - centro_y);
    _k_neg_y = 1.0f / centro_y;

    // Filtros y Espacialidad
    _k_deadzone = 1.0f / (1.0f - zona_muerta);
    _k_EMA_comp = 1.0f - _alpha;
}

float Joystick::ProcesarLectura(uint32_t valor_adc, float centro, float k_pos, float k_neg, float& estado_previo) {
    float valor_flotante = static_cast<float>(valor_adc);

	// 1. Normalizacion asimetrica
	float N = (valor_flotante >= centro) ?
			(valor_flotante - centro) * k_pos :
			(valor_flotante - centro) * k_neg;

    // 2. Saturacion (Hardware limites bounding)
	N = (N > 1.0f) ? 1.0f : ((N < -1.0f) ? -1.0f : N);

	// 3. Compensacion de Zona Muerta lineal
    float target = 0.0f;
    if (N > zona_muerta) {
    	target = (N - zona_muerta) * _k_deadzone;
    } else if (N < -zona_muerta) {
    	target = (N + zona_muerta) * _k_deadzone;
    }

    // 4. Mapeo No Lineal (Interpolar entre respuesta lineal y cúbica)
    if (target != 0.0f) {
        float cubo = target * target * target;
        target = (1.0f - _curvatura) * target + _curvatura * cubo;
    }

    // 5. Filtro EMA
    estado_previo = (_alpha * target) + (_k_EMA_comp * estado_previo);

    // Retorno continuo al dominio [-1.0f, 1.0f]
    return estado_previo;
}

void Joystick::Actualizar() {
	if (ptr_raw_x != nullptr && ptr_raw_y != nullptr) {
		raw_x = *ptr_raw_x;
		raw_y = *ptr_raw_y;

		valor_x = ProcesarLectura(raw_x, centro_x, _k_pos_x, _k_neg_x, _estado_filtrado_x);
		valor_y = ProcesarLectura(raw_y, centro_y, _k_pos_y, _k_neg_y, _estado_filtrado_y);
	}
}

void Joystick::NotificarInterrupcion(uint16_t pin_disparado) {
	// FSM: Transición iniciada exclusivamente por la ISR (Flujo unidireccional)
	if (pin_disparado == pin_sw && !en_proceso_debounce){
		tiempo_inicio_debounce = HAL_GetTick();
		en_proceso_debounce = true;
	}
}

bool Joystick::HayPulsacionValida() {
    if (en_proceso_debounce) {
    	// Evaluación no bloqueante de la ventana temporal
        if ((HAL_GetTick() - tiempo_inicio_debounce) >= RETARDO_ANTIRREBOTE_MS) {

        	// FSM: Transición finalizada exclusivamente por el bucle principal
            en_proceso_debounce = false;

            // Validación del estado eléctrico post-ventana
            if (HAL_GPIO_ReadPin(puerto_sw, pin_sw) == GPIO_PIN_RESET) {
            	return true;
            }
        }
    }
    return false;
}

float Joystick::ObtenerX() const { return valor_x; }
float Joystick::ObtenerY() const { return valor_y; }
uint32_t Joystick::ObtenerRawX() const { return raw_x; }
uint32_t Joystick::ObtenerRawY() const { return raw_y; }

void Joystick::setFiltroAlpha(float alpha) {
    _alpha = (alpha < 0.01f) ? 0.01f : ((alpha > 1.0f) ? 1.0f : alpha);
    _k_EMA_comp = 1.0f - _alpha; // Actualización del parámetro correlacionado
}

void Joystick::setCurvatura(float curvatura) {
    // Limitar la curvatura entre 0.0 (puramente lineal) y 1.0 (puramente cúbica)
    _curvatura = (curvatura < 0.0f) ? 0.0f : ((curvatura > 1.0f) ? 1.0f : curvatura);
}
