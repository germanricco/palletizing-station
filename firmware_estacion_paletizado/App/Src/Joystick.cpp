/**
 * @file Joystick.cpp
 * @brief Implementación de la fachada Joystick (compone JoystickInput + JoystickFilter).
 */

#include "Joystick.hpp"

Joystick::Joystick(volatile uint32_t* direccion_memoria_x, volatile uint32_t* direccion_memoria_y,
		GPIO_TypeDef* puerto_sw_asignado, uint16_t pin_sw_asignado,
		float calibracion_centro_x, float calibracion_centro_y, float umbral_zona_muerta)
    : _input(direccion_memoria_x, direccion_memoria_y, puerto_sw_asignado, pin_sw_asignado),
	  _filtroX(calibracion_centro_x, umbral_zona_muerta),
	  _filtroY(calibracion_centro_y, umbral_zona_muerta),
	  raw_x(0), raw_y(0) {
}

void Joystick::Actualizar() {
	raw_x = _input.LeerRawX();
	raw_y = _input.LeerRawY();

	_filtroX.Procesar(raw_x);
	_filtroY.Procesar(raw_y);
}

void Joystick::NotificarInterrupcion(uint16_t pin_disparado) {
	_input.NotificarInterrupcion(pin_disparado);
}

bool Joystick::HayPulsacionValida() {
	return _input.HayPulsacionValida();
}

float Joystick::ObtenerX() const { return _filtroX.Valor(); }
float Joystick::ObtenerY() const { return _filtroY.Valor(); }
uint32_t Joystick::ObtenerRawX() const { return raw_x; }
uint32_t Joystick::ObtenerRawY() const { return raw_y; }

void Joystick::setFiltroAlpha(float alpha) {
    _filtroX.setAlpha(alpha);
    _filtroY.setAlpha(alpha);
}

void Joystick::setCurvatura(float curvatura) {
    _filtroX.setCurvatura(curvatura);
    _filtroY.setCurvatura(curvatura);
}
