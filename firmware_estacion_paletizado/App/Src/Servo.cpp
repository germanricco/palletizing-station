/**
 * @file Servo.cpp
 * @brief Implementación de la clase Servo. Control unificado de posicion y velocidad
 */

#include "Servo.hpp"

// Definicion de PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Servo::Servo(volatile uint32_t* ccr_register,
		float minAngle, float maxAngle,
		float minPulseUs, float maxPulseUs,
		float homeAngle, float maxSpeed)
    : _ccr_reg(ccr_register),
	  _minAngle(minAngle),
	  _controlMode(ServoControlMode::SmoothPosition)
{
	// Cálculo de la pendiente de interpolación lineal
    _angleToPulseFactor = (maxPulseUs - minPulseUs) / (maxAngle - minAngle);

	// Mapeo inicial
    float homePulseFloat = minPulseUs + (homeAngle - minAngle) * _angleToPulseFactor;

	// Escalamiento a Punto Fijo Q16.16
    _minPulse_Q16 = static_cast<int32_t>(minPulseUs * Q16_FACTOR);
    _maxPulse_Q16 = static_cast<int32_t>(maxPulseUs * Q16_FACTOR);
    _homePulse_Q16 = static_cast<int32_t>(homePulseFloat * Q16_FACTOR);
    _maxSpeed_Q16 = static_cast<int32_t>(maxSpeed * Q16_FACTOR);

	// Configuracion parametrica de la inercia del software
	_acceleration_Q16 = _maxSpeed_Q16 / 10;

	// El sistema asume el estado de reposo inicial
	_currentCCR_Q16 = _homePulse_Q16;
	_targetPulse_Q16 = _homePulse_Q16;
	_targetVelocity_Q16 = 0;
	_currentVelocity_Q16 = 0;
}

int32_t Servo::convertAngleToPulseQ16(float angleDegrees) const {
    // Reconstrucción del pulso base minPulseUs implícito en la ecuación
    float minPulseUs = static_cast<float>(_minPulse_Q16) / Q16_FACTOR;
    float pulseUs = minPulseUs + (angleDegrees - _minAngle) * _angleToPulseFactor;
    return static_cast<int32_t>(pulseUs * Q16_FACTOR);
}

void Servo::init() {
    goHome();
    if (_ccr_reg != nullptr) {
        *_ccr_reg = (static_cast<uint32_t>(_homePulse_Q16) + Q16_HALF_STEP) >> 16;
    }
}

void Servo::goHome(){
	_controlMode = ServoControlMode::SmoothPosition;
	_targetPulse_Q16 = _homePulse_Q16;

	// Purgado de inercias
	_targetVelocity_Q16 = 0;
	_currentVelocity_Q16 = 0;
}

void Servo::setTargetVelocity(float normalizedVelocity) {
	_controlMode = ServoControlMode::Velocity;

	// Saturación de la entrada
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

void Servo::setSmoothTargetPosition(float angleDegrees) {
    _controlMode = ServoControlMode::SmoothPosition;

    // Purgado de Inercias
    _targetVelocity_Q16 = 0;
    _currentVelocity_Q16 = 0;

    int64_t target_Intermediate = convertAngleToPulseQ16(angleDegrees);

    // Saturación preventiva de posicion objetivo
    if (target_Intermediate > _maxPulse_Q16) _targetPulse_Q16 = _maxPulse_Q16;
    else if (target_Intermediate < _minPulse_Q16) _targetPulse_Q16 = _minPulse_Q16;
    else _targetPulse_Q16 = static_cast<int32_t>(target_Intermediate);
}

void Servo::setDirectTargetPosition(float angleDegrees) {
    _controlMode = ServoControlMode::DirectPosition;

    // Purgado de inercias
    _targetVelocity_Q16 = 0;
    _currentVelocity_Q16 = 0;

    int64_t target_Intermediate = convertAngleToPulseQ16(angleDegrees);

    int32_t valid_target;
    if (target_Intermediate > _maxPulse_Q16) valid_target = _maxPulse_Q16;
    else if (target_Intermediate < _minPulse_Q16) valid_target = _minPulse_Q16;
    else valid_target = static_cast<int32_t>(target_Intermediate);
    
    // Se actualiza la posicion actual y la consigna
    _currentCCR_Q16 = valid_target;
    _targetPulse_Q16 = valid_target;
}

void Servo::update() {
	// Barrera de seguridad
	if(_ccr_reg == nullptr) {
		return;
	}

	// Resolucion de la maquina de estados
	if (_controlMode == ServoControlMode::Velocity) {
        // Prediccion de Colision (Soft Limits Direccionales)
        int32_t absVelocity_Q16 = (_currentVelocity_Q16 < 0) ? -_currentVelocity_Q16 : _currentVelocity_Q16;
        
        int32_t steps_to_stop = (absVelocity_Q16 + _acceleration_Q16 - 1) / _acceleration_Q16;
        int32_t braking_distance_Q16 = (steps_to_stop * absVelocity_Q16 + absVelocity_Q16) / 2;

        int32_t dist_to_max = _maxPulse_Q16 - _currentCCR_Q16;
        int32_t dist_to_min = _currentCCR_Q16 - _minPulse_Q16;

        int32_t safe_target_velocity = _targetVelocity_Q16;

        // Override Direccional: 
        // Solo forzamos a 0 si la intención del usuario empuja HACIA el límite
        if (_currentVelocity_Q16 > 0 && dist_to_max <= braking_distance_Q16) {
            if (safe_target_velocity > 0) {
                safe_target_velocity = 0; 
            }
        } else if (_currentVelocity_Q16 < 0 && dist_to_min <= braking_distance_Q16) {
            if (safe_target_velocity < 0) {
                safe_target_velocity = 0; 
            }
        }

        // Rampa de Velocidad Cinematica (Slew Rate Limiter)
        if (_currentVelocity_Q16 < safe_target_velocity) {
            _currentVelocity_Q16 += _acceleration_Q16;
            if (_currentVelocity_Q16 > safe_target_velocity) _currentVelocity_Q16 = safe_target_velocity;
        } else if (_currentVelocity_Q16 > safe_target_velocity) {
            _currentVelocity_Q16 -= _acceleration_Q16;
            if (_currentVelocity_Q16 < safe_target_velocity) _currentVelocity_Q16 = safe_target_velocity;
        }

        // Actualizacion de Estado (Con Hard Clamp)
        int64_t next_ccr = static_cast<int64_t>(_currentCCR_Q16) + _currentVelocity_Q16;

        // El Hard Clamp es vital para evitar desbordes en caso de comandos extremos
        if (next_ccr > _maxPulse_Q16) {
            _currentCCR_Q16 = _maxPulse_Q16;
            _currentVelocity_Q16 = 0; // Purgamos la inercia instantáneamente
        } else if (next_ccr < _minPulse_Q16) {
            _currentCCR_Q16 = _minPulse_Q16;
            _currentVelocity_Q16 = 0;
        } else {
            _currentCCR_Q16 = static_cast<int32_t>(next_ccr);
        }
        
        // Sincronizamos el target de posición para mantener la consistencia del estado
        _targetPulse_Q16 = _currentCCR_Q16; 

    } else if (_controlMode == ServoControlMode::SmoothPosition) {
        int32_t error_Q16 = _targetPulse_Q16 - _currentCCR_Q16;
        int32_t absError_Q16 = (error_Q16 < 0) ? -error_Q16 : error_Q16;

        if (absError_Q16 <= DEADBAND_Q16) {
            _currentCCR_Q16 = _targetPulse_Q16;
            _currentVelocity_Q16 = 0; // Detención total
        } else {
            // Cinemática Predictiva (Perfil Trapezoidal Simétrico)
            int32_t absVelocity_Q16 = (_currentVelocity_Q16 < 0) ? -_currentVelocity_Q16 : _currentVelocity_Q16;

            // Calculo de frenado. Redondeo hacia arriba para asegurar que siempre se frene a tiempo
            int32_t steps_to_stop = (absVelocity_Q16 + _acceleration_Q16 - 1) / _acceleration_Q16;
            
            // Suma aritmetica escalonada
            int32_t braking_distance_Q16 = (steps_to_stop * absVelocity_Q16 + absVelocity_Q16) / 2;

            int32_t desired_velocity = 0;

            // Lógica de Navegación Bang-Bang
            if (absError_Q16 <= braking_distance_Q16) {
                // Entramos a zona de frenado, ordenamos detenernos
                desired_velocity = 0; 
            } else {
                // Aceleracion a velocidad constante
                desired_velocity = (error_Q16 > 0) ? _maxSpeed_Q16 : -_maxSpeed_Q16;
            }

            // Zona de Aterrizaje suave (Anti-Chattering)
            // Actua como controlador P/4 que actua solo si la velocidad 
            // pedida por el Bang-Bang nos haria pasarnos del objetivo.
            int32_t safe_velocity = error_Q16 / 4;

            // Aplicamos el límite seguro manteniendo la intención direccional
            if (desired_velocity > 0 && desired_velocity > safe_velocity && safe_velocity > 0) {
                desired_velocity = safe_velocity;
            } else if (desired_velocity < 0 && desired_velocity < safe_velocity && safe_velocity < 0) {
                desired_velocity = safe_velocity;
            }

            // Slew Rate Limiter (Forzamos aceleración y deceleración constantes y simétricas)
            if (desired_velocity > _currentVelocity_Q16 + _acceleration_Q16) {
                _currentVelocity_Q16 += _acceleration_Q16;
            } else if (desired_velocity < _currentVelocity_Q16 - _acceleration_Q16) {
                _currentVelocity_Q16 -= _acceleration_Q16;
            } else {
                _currentVelocity_Q16 = desired_velocity;
            }

            // Empuje residual para vencer friccion estatica (Deadband)
            if (_currentVelocity_Q16 == 0 && desired_velocity != 0) {
                 _currentVelocity_Q16 = (error_Q16 > 0) ? 13107 : -13107; 
            }

            _currentCCR_Q16 += _currentVelocity_Q16;
        }
    }

    // Recorte estricto para cumplir limites mecanicos
    if (_currentCCR_Q16 > _maxPulse_Q16) {
        _currentCCR_Q16 = _maxPulse_Q16;
        _currentVelocity_Q16 = 0;
    }
    else if (_currentCCR_Q16 < _minPulse_Q16) {
        _currentCCR_Q16 = _minPulse_Q16;
        _currentVelocity_Q16 = 0;
    }

    // Ejecucion en registro hardware
    uint32_t ccr_unsigned = static_cast<uint32_t>(_currentCCR_Q16);
    *_ccr_reg = (ccr_unsigned + Q16_HALF_STEP) >> 16;
}

void Servo::setMaxSpeed(float newMaxSpeed) {
    if (newMaxSpeed > 0.01f) {
    	_maxSpeed_Q16 = static_cast<int32_t>(newMaxSpeed * Q16_FACTOR);
    	_acceleration_Q16 = _maxSpeed_Q16 / 10;
    }
}

float Servo::getAngleDegrees() const {
	float minPulseUs = static_cast<float>(_minPulse_Q16) / Q16_FACTOR;
    float currentPulseUs = static_cast<float>(_currentCCR_Q16) / Q16_FACTOR;
    return _minAngle + (currentPulseUs - minPulseUs) / _angleToPulseFactor;
}

float Servo::getAngleRadians() const {
	// (PI / Rango) * (ValorActual - DesplazamientoMinimo
	return getAngleDegrees() * (M_PI / 180.0f);
}

bool Servo::isTargetReached() const {
    if (_controlMode == ServoControlMode::Velocity) {
        return false; // En modo velocidad pura no hay un destino absoluto
    }

    int32_t error_Q16 = _targetPulse_Q16 - _currentCCR_Q16;
    int32_t absError_Q16 = (error_Q16 < 0) ? -error_Q16 : error_Q16;

    // Si el error es menor o igual a la banda muerta (0.5 us), llegamos.
    return (absError_Q16 <= DEADBAND_Q16);
}
