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
	constexpr float SERVO_BASE_MAX_SPEED = 16.0f;
	constexpr int32_t SERVO_BASE_ACCEL_DIVISOR = 12;

	// Articulacion 2: Hombro
	constexpr float SERVO_HOMBRO_MIN_ANGLE = 40.0f;
	constexpr float SERVO_HOMBRO_MAX_ANGLE = 120.0f;
	constexpr float SERVO_HOMBRO_HOME_ANGLE = 80.0f;
	constexpr float SERVO_HOMBRO_MIN_PULSE_US = 500.0f;
	constexpr float SERVO_HOMBRO_MAX_PULSE_US = 2500.0f;
	constexpr float SERVO_HOMBRO_MAX_SPEED = 8.0f;
	constexpr int32_t SERVO_HOMBRO_ACCEL_DIVISOR = 16;

	// Articulacion 3: Codo
	constexpr float SERVO_CODO_MIN_ANGLE = 45.0f;
	constexpr float SERVO_CODO_MAX_ANGLE = 180.0f;
	constexpr float SERVO_CODO_HOME_ANGLE = 100.0f;
	constexpr float SERVO_CODO_MIN_PULSE_US = 500.0f;
	constexpr float SERVO_CODO_MAX_PULSE_US = 2500.0f;
	constexpr float SERVO_CODO_MAX_SPEED = 10.0f;
	constexpr int32_t SERVO_CODO_ACCEL_DIVISOR = 8;

	// Actuador

	/**
	 * ======== CINTA TRANSPORTADORA ========
	 */
	// Velocidad nominal normalizada [0.0 a 1.0]. Con TIM2 ARR=49 (PWM 20kHz, 50 pasos) y
	// CINTA_MIN_CCR=0, MotorDC trunca v*49: 0.25 -> CCR=12 (24%), 0.27 -> CCR=13 (26%).
	// El 25% exacto no es representable (seria CCR=12.5).
	constexpr float CINTA_VELOCIDAD_NOMINAL = 0.27f;

	// Duracion de la rampa lineal de arranque. Evita que la caja patine y limita el pico de
	// corriente de arranque del motor. El frenado es siempre inmediato (precision de parada).
	constexpr uint32_t CINTA_RAMPA_ARRANQUE_MS = 300U;

	// Umbral minimo de CCR para vencer la friccion estatica (ver MotorDC). Ojo: modificarlo
	// cambia el duty efectivo de CINTA_VELOCIDAD_NOMINAL.
	constexpr uint32_t CINTA_MIN_CCR = 0U;

	// Antirrebote del sensor IR de fin de cinta (FC-51, activo en bajo, PB12).
	constexpr uint32_t SENSOR_IR_DEBOUNCE_MS = 30U;

	// Modo de prueba aislada de la clase Cinta (sin StationController). Solo para bring-up.
	constexpr bool CINTA_BRINGUP = true;

	/**
	 * ======== CALIBRACION DE ENTRADAS ANALOGICAS (JOYSTICKS) ========
	 */
	constexpr float ADC_MAX_RESOLUTION = 4095.0f;

	// Parametros de procesamiento digital de senales (DSP)
	constexpr float JOYSTICK_DEADZONE = 0.05f;        // 5% de zona muerta central

	// Tasa fija de muestreo/filtrado del joystick. Se fija a
	// 100 Hz: el doble del tick de actuacion del servo (50 Hz / 20 ms, ver TIM3), cumpliendo
	// Nyquist con margen respecto al consumidor de la consigna de velocidad.
	constexpr uint32_t JOYSTICK_SAMPLE_PERIOD_MS = 10U;

	// Factor de filtro EMA (Suavizado). Con Ts = JOYSTICK_SAMPLE_PERIOD_MS fijo, la constante de
	// tiempo del filtro queda definida por alpha = 1 - exp(-Ts/tau). Con Ts = 10 ms este valor
	// corresponde a tau ~= 30 ms (fc ~= 1/(2*pi*tau) ~= 5.3 Hz), similar al suavizado original
	// pero ahora con una frecuencia de corte determinista.
	constexpr float JOYSTICK_EMA_ALPHA = 0.28f;

	constexpr float JOYSTICK_CURVATURE = 0.0f;

	// Valores de reposo (Centro) calibrados estáticamente
	constexpr float JOYSTICK_IZQUIERDO_X_CENTER = 1958.0f;
	constexpr float JOYSTICK_IZQUIERDO_Y_CENTER = 1991.0f;
	constexpr float JOYSTICK_DERECHO_X_CENTER = 2000.0f;
	constexpr float JOYSTICK_DERECHO_Y_CENTER = 2000.0f;
} // namespace Config

#endif	/* CONFIG_HPP */
