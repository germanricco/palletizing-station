/**
 * @file Servo.cpp
 * @brief Implementación de la clase Servo.
 */

#include "Servo.hpp"

// Definicion de PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Servo::Servo(volatile uint32_t* ccr_register, float minAngle, float maxAngle,
		float homeAngle, float maxSpeed)
    : _ccr_reg(ccr_register),
	  _maxSpeed(maxSpeed),
	  _stepSize(0.0f){

	// Transformacion del dominio angular al dominio temporal (ancho de pulso)
	_minPulse = static_cast<uint32_t>((minAngle * 100.0f / 9.0f) + 500.0f);
	_maxPulse = static_cast<uint32_t>((maxAngle * 100.0f / 9.0f) + 500.0f);
	_homePulse = static_cast<uint32_t>((homeAngle * 100.0f / 9.0f) + 500.0f);

	// Precalculo de limites flotantes
	_minPulseFloat = static_cast<float>(_minPulse);
	_maxPulseFloat = static_cast<float>(_maxPulse);

	// El sistema asume el estado de reposo inicial
	_currentCCR = static_cast<float>(_homePulse);
}

void Servo::init() {
	_stepSize = 0.0f;
	_currentCCR = static_cast<float>(_homePulse);

	// Barrera de hardware (Hard Fault)
    if (_ccr_reg != nullptr) {
    	*_ccr_reg = _homePulse;
    }
}

void Servo::goHome(){
	_stepSize = 0.0f;
	_currentCCR = static_cast<float>(_homePulse);

	if (_ccr_reg != nullptr) {
	    *_ccr_reg = _homePulse; //AGREGAR VELOCIDAD 10 o 20 PULSOS, QUE NO SEA A LA VELOCIDAD
	}
}

void Servo::setVelocityCommand(float velocityFactor) {
    float velocidadTeorica = velocityFactor * _maxSpeed;

    // Cálculo eficiente de valor absoluto in-line (reemplaza std::abs)
    float absVelocidad = (velocidadTeorica < 0.0f) ? -velocidadTeorica : velocidadTeorica;

    // Filtrado de ruido e intenciones de detención
    if (absVelocidad < 0.001f) {
    	_stepSize = 0.0f;
        return;
    }

    _stepSize = velocidadTeorica;
}

void Servo::updatePosition() {
	// Barrera de hardware de hot-path
    if (_ccr_reg != nullptr) {
        _currentCCR += _stepSize;

        // Saturacion en los limites fisicos
        if (_currentCCR > _maxPulseFloat) {
            _currentCCR = _maxPulseFloat;
        } else if (_currentCCR < _minPulseFloat) {
            _currentCCR = _minPulseFloat;
        }

        // Ejecucion en registro
        *_ccr_reg = static_cast<uint32_t>(_currentCCR + 0.5f);
    }
}

void Servo::setMaxSpeed(float newMaxSpeed) {
    if (newMaxSpeed > 0.01f) {
        _maxSpeed = newMaxSpeed;
    }
}

float Servo::getAngleDegrees() const {
	return (_currentCCR - 500.0f) * 0.09f;
}

float Servo::getAngleRadians() const {
	// (PI / Rango) * (ValorActual - DesplazamientoMinimo
	return (_currentCCR - 500.0f) * (M_PI / 2000.0f);
}
