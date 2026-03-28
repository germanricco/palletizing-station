/**
 * @file Servo.cpp
 * @brief Implementación de la clase Servo.
 */

#include "Servo.hpp"

// Definicion de PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Servo::Servo(volatile uint32_t* ccr_register, uint32_t minP, uint32_t maxP,
		uint32_t homeP, float minSpeed, float maxSpeed)
    : _ccr_reg(ccr_register),
      _minPulse(minP),
      _maxPulse(maxP),
	  _homePulse(homeP),
	  _minSpeed(minSpeed),
	  _maxSpeed(maxSpeed),
	  _stepSize(0.0f){

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
	    *_ccr_reg = _homePulse;
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

    // Insercion de la velocidad umbral minima con preservacion de signo
    if (absVelocidad < _minSpeed){
    	_stepSize = (velocidadTeorica < 0.0f) ? -_minSpeed : _minSpeed;
    } else {
    	_stepSize = velocidadTeorica;
    }
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
        *_ccr_reg = static_cast<uint32_t>(_currentCCR);
    }
}

void Servo::setMaxSpeed(float newMaxSpeed) {
    if (newMaxSpeed > 0.01f) {
        _maxSpeed = newMaxSpeed;

        // Prevencion de estados ilogicos de configuracion
        if (_minSpeed > _maxSpeed) {
        	_minSpeed = _maxSpeed * 0.01f;
        }
    }
}

void Servo::setMinSpeed(float newMinSpeed) {
    if (newMinSpeed >= 0.0f && newMinSpeed < _maxSpeed) {
    	_minSpeed = newMinSpeed;
    }
}

float Servo::getAngleDegrees() const {
    // (180.0f / Rango) * (ValorActual - DesplazamientoMinimo)
    return (180.0f / (_maxPulseFloat - _minPulseFloat)) * (_currentCCR - _minPulseFloat);
}

float Servo::getAngleRadians() const {
	// (PI / Rango) * (ValorActual - DesplazamientoMinimo
    return (M_PI / (_maxPulseFloat - _minPulseFloat)) * (_currentCCR - _minPulseFloat);
}
