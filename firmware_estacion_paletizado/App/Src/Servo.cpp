/**
 * @file Servo.cpp
 * @brief Implementación de la clase Servo. Control unificado de posicion y velocidad
 */

#include "Servo.hpp"

// Definicion de PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Servo::Servo(volatile uint32_t* ccr_register, float minAngle, float maxAngle,
		float homeAngle, float maxSpeed)
    : _ccr_reg(ccr_register),
	  _controlMode(ServoControlMode::SmoothPosition) {

	// Interpolación lineal absoluta a microsegundos [180 grados = span de 2000us]
	// Pulse(angle) = theta*(2500-500)/(180-0) + 500 = theta * 11.11... + 500
	// Ej. si minAngle = 90.0f -> minPulseFloat = 1500.0f
	float minPulseFloat = (minAngle * 11.111111f) + 500.0f;
	float maxPulseFloat = (maxAngle * 11.111111f) + 500.0f;
	float homePulseFloat = (homeAngle * 11.111111f) + 500.0f;

	// Escalamiento a Punto Fijo Q16.16
	// Realizamos desplazamiento de 16bits a la izquierda multiplicando por 2^16
	_minPulse_Q16 = static_cast<int32_t>(minPulseFloat * 65536.0f);
	_maxPulse_Q16 = static_cast<int32_t>(maxPulseFloat * 65536.0f);
	_homePulse_Q16 = static_cast<int32_t>(homePulseFloat * 65536.0f);
	_maxSpeed_Q16 = static_cast<int32_t>(maxSpeed * 65536.0f);

	// Configuracion parametrica de la inercia del software
	_acceleration_Q16 = _maxSpeed_Q16 / 10;

	// El sistema asume el estado de reposo inicial
	_currentCCR_Q16 = _homePulse_Q16;
	_targetPulse_Q16 = _homePulse_Q16;
	_targetVelocity_Q16 = 0;
	_currentVelocity_Q16 = 0;
}

void Servo::init() {
	_targetVelocity_Q16 = 0;
	_currentVelocity_Q16 = 0;
	_currentCCR_Q16 = _homePulse_Q16;
	_targetPulse_Q16 = _homePulse_Q16;
	_controlMode = ServoControlMode::SmoothPosition;

	// Barrera de hardware (Hard Fault)
    if (_ccr_reg != nullptr) {
    	// Desplazamiento al hardware
    	// Se fuerza la interpretacion del bloque de memoria como entero
    	// para que >> se compile como desplazamiento logico
    	// +32768U para hacer redonedo (sumar 0.5 y luego truncar)
    	*_ccr_reg = (static_cast<uint32_t>(_homePulse_Q16) + 32768U) >> 16;
    }
}

void Servo::goHome(){
	_controlMode = ServoControlMode::SmoothPosition;
	_targetPulse_Q16 = _homePulse_Q16;
}

void Servo::commandVelocity(float normalizedVelocity) {
	_controlMode = ServoControlMode::Velocity;

	// Saturación de la entrada para evitar desbordamientos de intención direccional
	if (normalizedVelocity > 1.0f) normalizedVelocity = 1.0f;
	else if (normalizedVelocity < -1.0f) normalizedVelocity = -1.0f;

	// Emulación del valor absoluto in-line
	float absVelocidad = (normalizedVelocity < 0.0f) ? -normalizedVelocity : normalizedVelocity;

    // Filtrado de ruido e intenciones de detención
    if (absVelocidad < 0.001f) {
    	_targetVelocity_Q16 = 0;
        return;
    }

    _targetVelocity_Q16 = static_cast<int32_t>(normalizedVelocity * static_cast<float>(_maxSpeed_Q16));
}

void Servo::commandSmoothPosition(float angleDegrees) {
    _controlMode = ServoControlMode::SmoothPosition;

    // Conversion a microsegundos en punto flotante
    float targetPulseFloat = (angleDegrees * 11.111111f) + 500.0f;
    int64_t target_Intermediate = static_cast<int64_t>(targetPulseFloat * 65536.0f);

    // Saturación preventiva contra colisiones antes de iniciar el desplazamiento
    if (target_Intermediate > static_cast<int64_t>(_maxPulse_Q16)) {
        _targetPulse_Q16 = _maxPulse_Q16;
    } else if (target_Intermediate < static_cast<int64_t>(_minPulse_Q16)) {
        _targetPulse_Q16 = _minPulse_Q16;
    } else {
        _targetPulse_Q16 = static_cast<int32_t>(target_Intermediate);
    }
}

void Servo::commandDirectPosition(float angleDegrees) {
    _controlMode = ServoControlMode::DirectPosition;

    float targetPulseFloat = (angleDegrees * 11.111111f) + 500.0f;
    int64_t target_Intermediate = static_cast<int64_t>(targetPulseFloat * 65536.0f);

    // Inyección directa a la variable de estado (ZOH transparente)
    if (target_Intermediate > static_cast<int64_t>(_maxPulse_Q16)) {
        _currentCCR_Q16 = _maxPulse_Q16;
    } else if (target_Intermediate < static_cast<int64_t>(_minPulse_Q16)) {
        _currentCCR_Q16 = _minPulse_Q16;
    } else {
        _currentCCR_Q16 = static_cast<int32_t>(target_Intermediate);
    }
}

void Servo::update() {
	// Barrera de seguridad
	if(_ccr_reg == nullptr) {
		return;
	}

	// Resolucion de la maquina de estados
	if (_controlMode == ServoControlMode::Velocity){
		// Limitador de tasa de giro (Slew Rate Limiter)
		if (_currentVelocity_Q16 < _targetVelocity_Q16) {
			// Aumento la velocidad
			_currentVelocity_Q16 += _acceleration_Q16;
			// Valido que esta nueva velocidad no supere el target
			if (_currentVelocity_Q16 > _targetVelocity_Q16) _currentVelocity_Q16 = _targetVelocity_Q16;
		} else if (_currentVelocity_Q16 > _targetVelocity_Q16) {
		    _currentVelocity_Q16 -= _acceleration_Q16;
		    if (_currentVelocity_Q16 < _targetVelocity_Q16) _currentVelocity_Q16 = _targetVelocity_Q16;
		}

		_currentCCR_Q16 += _currentVelocity_Q16;

	} else if (_controlMode == ServoControlMode::SmoothPosition) {
		int32_t error_Q16 = _targetPulse_Q16 - _currentCCR_Q16;
	    int32_t absError_Q16 = (error_Q16 < 0) ? -error_Q16 : error_Q16;

	    // Banda muerta (Deadband) de 0.5 us para eliminar micro-oscilaciones asintóticas
        if (absError_Q16 <= 32768) {
            _currentCCR_Q16 = _targetPulse_Q16;
        } else {
            // Desaceleración Proporcional: Se acerca suavemente al objetivo (/8 actúa como Kp)
            int32_t step = error_Q16 / 8;

            if (step > _maxSpeed_Q16) step = _maxSpeed_Q16;
            else if (step < -_maxSpeed_Q16) step = -_maxSpeed_Q16;

            // Garantizar un paso mínimo para no estancarse antes de la banda muerta
            if (step == 0) step = (error_Q16 > 0) ? 13107 : -13107; // ~0.2 us de empuje residual

            _currentCCR_Q16 += step;
        }
    } else if (_controlMode == ServoControlMode::DirectPosition) {
    	//
    }

	// Recorte estricto para cumplir limites mecanicos
	if (_currentCCR_Q16 > _maxPulse_Q16) {
		_currentCCR_Q16 = _maxPulse_Q16;
	} else if (_currentCCR_Q16 < _minPulse_Q16) {
	    _currentCCR_Q16 = _minPulse_Q16;
	}

	// Ejecucion en registro con compensacion de sesgo de truncamiento
	// 1. Casting a Unsigned: Garantiza desplazamiento lógico y no aritmético por el compilador.
	// 2. Suma de +32768U: Equivale a +0.5 en notación de coma flotante (Redondeo geométrico).
	// 3. Shift >> 16: Desescala Q16.16 devolviendo la magnitud entera de microsegundos al registro.
	uint32_t ccr_unsigned = static_cast<uint32_t>(_currentCCR_Q16);
	*_ccr_reg = (ccr_unsigned + 32768U) >> 16;
}

void Servo::setMaxSpeed(float newMaxSpeed) {
    if (newMaxSpeed > 0.01f) {
    	_maxSpeed_Q16 = static_cast<int32_t>(newMaxSpeed * 65536.0f);
    	_acceleration_Q16 = _maxSpeed_Q16 / 10;
    }
}

float Servo::getAngleDegrees() const {
	float ccr_float = static_cast<float>(_currentCCR_Q16) / 65536.0f;
	return (ccr_float - 500.0f) * 0.09f;
}

float Servo::getAngleRadians() const {
	// (PI / Rango) * (ValorActual - DesplazamientoMinimo
	float ccr_float = static_cast<float>(_currentCCR_Q16) / 65536.0f;
	return (ccr_float - 500.0f) * (M_PI / 2000.0f);
}
