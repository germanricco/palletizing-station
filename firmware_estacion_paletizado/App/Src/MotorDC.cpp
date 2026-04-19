/**
 * @file MotorDC.cpp
 * @brief Implementación dpara MotorDC
 */

#include "MotorDC.hpp"

MotorDC::MotorDC(volatile uint32_t* ccr_register, uint32_t arr_max, uint32_t min_ccr,
		float input_deadband, MotorBrakeMode brake_mode,
		GPIO_TypeDef* port_in1, uint16_t pin_in1,
		GPIO_TypeDef* port_in2, uint16_t pin_in2)
    : _ccr_reg(ccr_register),
      _arr_max_value(arr_max),
	  _min_ccr_value(min_ccr),
      _port_in1(port_in1),
      _port_in2(port_in2),
      _currentSpeed_Q16(0),
	  _brake_mode(brake_mode) {

	// Sanidad de hardware y precalculo de memoria
	if (_min_ccr_value > _arr_max_value) {
		_min_ccr_value = _arr_max_value;
	}
	_dynamic_ccr_range = _arr_max_value - _min_ccr_value;

	if (input_deadband < 0.0f) input_deadband = 0.0f;
	_input_deadband_Q16 = static_cast<int32_t>(input_deadband * 65536.0f);

	// Deteccion de topologia
	_is_bidirectional = (_port_in1 != nullptr && _port_in2 != nullptr);

	// Precalculo de máscaras BSRR (Bit Set/Reset Register)
    // En arquitecturas STM32F1, los 16 bits inferiores hacen SET, los 16 superiores hacen RESET.
    if (_is_bidirectional) {
        _pin_in1_set = pin_in1;
        _pin_in1_reset = static_cast<uint32_t>(pin_in1) << 16;
        _pin_in2_set = pin_in2;
        _pin_in2_reset = static_cast<uint32_t>(pin_in2) << 16;
    }
}

void MotorDC::init() {
    setSpeedQ16(0); // Fuerza un arranque seguro en 0 RPM
}

void MotorDC::setSpeed(float normalized_speed) {
	// Interfaz publica flotante (Redirije hacia el nucleo Q16.16
	if (normalized_speed > 1.0f) normalized_speed = 1.0f;
	else if (normalized_speed < -1.0f) normalized_speed = -1.0f;

	setSpeedQ16(static_cast<int32_t>(normalized_speed * 65536.0f));
}

void MotorDC::setSpeedQ16(int32_t speed_Q16) {
	_currentSpeed_Q16 = speed_Q16;

	// Topologia de direccion
	if (_is_bidirectional) {
        if (_currentSpeed_Q16 > _input_deadband_Q16) { // Tolerancia de zona muerta analógica (~0.001)
            _port_in1->BSRR = _pin_in1_set;
            _port_in2->BSRR = _pin_in2_reset;
        } else if (_currentSpeed_Q16 < -_input_deadband_Q16) {
            _port_in1->BSRR = _pin_in1_reset;
            _port_in2->BSRR = _pin_in2_set;
        } else {
            // Lógica de mitigación inercial
            if (_brake_mode == MotorBrakeMode::Brake) {
                _port_in1->BSRR = _pin_in1_set;
                _port_in2->BSRR = _pin_in2_set;
            } else {
                _port_in1->BSRR = _pin_in1_reset;
                _port_in2->BSRR = _pin_in2_reset;
            }
        }
    } else {
        // Si se comanda velocidad negativa a un MOSFET simple se fuerza la detencion.
        if (_currentSpeed_Q16 < 0) {
            _currentSpeed_Q16 = 0;
        }
    }

	// RESOLUCION DE TRANSFERENCIA DE POTENCIA
	if (_ccr_reg != nullptr) {
		// Extraccion de magnitud
		int32_t abs_speed_Q16 = (_currentSpeed_Q16 < 0) ? -_currentSpeed_Q16 : _currentSpeed_Q16;
		uint32_t compare_value = 0;

		// Evaluacion con banda muerta de entrada
		if (abs_speed_Q16 > _input_deadband_Q16) {
			// Duty = MinDuty + (Speed * RangoUtil)
			compare_value = _min_ccr_value + static_cast<uint32_t>((abs_speed_Q16 * _dynamic_ccr_range) >> 16);

			// Saturacion
			if (compare_value > _arr_max_value) {
				compare_value = _arr_max_value;
			}
		} else {
			compare_value = 0;
		}

		// Inyeccion al registro de hardware
		*_ccr_reg = compare_value;
	}

}





