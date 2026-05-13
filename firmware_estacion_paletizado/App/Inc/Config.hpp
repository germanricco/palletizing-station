/**
 * @file Config.hpp
 * #brief Archivo central de configuracion estatica para el firmware
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <stdint.h>
#include "Logger.hpp"

namespace Config {
	/**
	 * ======== CONFIGURACION DEL SISTEMA Y COMUNICACIONES
	 */
	constexpr LogLevel DEFAULT_LOG_LEVEL = LogLevel::Info;

	/**
	 * ======== LIMITES Y CALIBRACION DE SERVOMOTORES ========
	 */
	// Articulacion 1: Base
	constexpr float SERVO_BASE_MIN_ANGLE = 0.0f;
	constexpr float SERVO_BASE_MAX_ANGLE = 180.0f;
	constexpr float SERVO_BASE_HOME_ANGLE = 90.0f;
	constexpr float SERVO_BASE_MIN_PULSE_US = 500.0f;
	constexpr float SERVO_BASE_MAX_PULSE_US = 2500.0f;
	constexpr float SERVO_BASE_MAX_SPEED = 20.0f;

	// Articulacion 2: Hombro
	constexpr float SERVO_HOMBRO_MIN_ANGLE = 40.0f;
	constexpr float SERVO_HOMBRO_MAX_ANGLE = 120.0f;
	constexpr float SERVO_HOMBRO_HOME_ANGLE = 80.0f;
	constexpr float SERVO_HOMBRO_MIN_PULSE_US = 500.0f;
	constexpr float SERVO_HOMBRO_MAX_PULSE_US = 2500.0f;
	constexpr float SERVO_HOMBRO_MAX_SPEED = 6.0f;

	// Articulacion 3: Codo
	constexpr float SERVO_CODO_MIN_ANGLE = 45.0f;
	constexpr float SERVO_CODO_MAX_ANGLE = 180.0f;
	constexpr float SERVO_CODO_HOME_ANGLE = 90.0f;
	constexpr float SERVO_CODO_MIN_PULSE_US = 500.0f;
	constexpr float SERVO_CODO_MAX_PULSE_US = 2500.0f;
	constexpr float SERVO_CODO_MAX_SPEED = 12.0f;

	// Actuador

	/**
	 * ======== CALIBRACION DE ENTRADAS ANALOGICAS (JOYSTICKS) ========
	 */
	constexpr float ADC_MAX_RESOLUTION = 4095.0f;

	// Parametros de procesamiento digital de senales (DSP)
	constexpr float JOYSTICK_DEADZONE = 0.05f;        // 5% de zona muerta central
	constexpr float JOYSTICK_EMA_ALPHA = 0.8f;        // Factor de filtro de promedio móvil (Suavizado)

	//! ========== CORREGIR  !! ===========
	constexpr float JOYSTICK_RESOLUTION_STEPS = 10.0f;// Discretización a 10 niveles de velocidad

	// Valores de reposo (Centro) calibrados estáticamente
	constexpr float JOYSTICK_IZQUIERDO_X_CENTER = 1900.0f;
	constexpr float JOYSTICK_IZQUIERDO_Y_CENTER = 1900.0f;
	constexpr float JOYSTICK_DERECHO_X_CENTER = 1900.0f;
	constexpr float JOYSTICK_DERECHO_Y_CENTER = 1900.0f;
} // namespace Config

#endif	/* CONFIG_HPP */
