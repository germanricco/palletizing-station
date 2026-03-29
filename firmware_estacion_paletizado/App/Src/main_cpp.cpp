#include "main_cpp.hpp"
#include "Config.hpp"
#include "Joystick.hpp"
#include "Servo.hpp"
#include "TerminalSerial.hpp"
#include "Logger.hpp"

#include "stm32f1xx_hal.h"
#include <cstdio>
#include <cstring>

extern "C" {
    extern ADC_HandleTypeDef hadc1;
    extern TIM_HandleTypeDef htim3;			// Dedicado al lazo cinetico (50Hz)
    extern UART_HandleTypeDef huart1;
}

// Arreglo global llenado por DMA
volatile uint32_t buffer_global_adc[4];

Joystick joystick_izquierdo(&buffer_global_adc[0], &buffer_global_adc[1],
		GPIOB, GPIO_PIN_5,
		Config::JOYSTICK_IZQUIERDO_X_CENTER,
		Config::JOYSTICK_IZQUIERDO_Y_CENTER,
		Config::JOYSTICK_DEADZONE);

Servo servo_base(&TIM3->CCR1,
		Config::SERVO_BASE_MIN_ANGLE,
		Config::SERVO_BASE_MAX_ANGLE,
		Config::SERVO_BASE_HOME_ANGLE,
		Config::SERVO_BASE_MAX_SPEED);


TerminalSerial terminal(&huart1);

// Variable global para la temporizacion por software
uint32_t tiempo_ultimo_envio_uart = 0;


void Cpp_Main_Init(void) {
	// Inicia interrupcion de recepcion de la terminal
	terminal.init();

	// Inicializacion de Logger utilizando terminal para envio de datos mediante DMA
	Logger::init(&terminal, Config::DEFAULT_LOG_LEVEL);
	Logger::logINFO("Inicializando sistema ...");

	// Inicia el escaneo DMA de ADC1 para los 2 canales simultaneamente
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);
	__HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

	// Inicializacion de Servos
	servo_base.init();
	//servo_hombro.init();
	//servo_muneca.init();

	// Inicializacion de PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim3);

    // Inicializar el temporizador global
    tiempo_ultimo_envio_uart = HAL_GetTick();

    HAL_Delay(2000);
}



void Cpp_Main_Loop(void) {
	// Actualizar estado interno de Joysticks
	joystick_izquierdo.Actualizar();
	//joystick_derecho.Actualizar();

	// Transferir la intencion al actuador (-1.0 a 1.0)
	servo_base.setVelocityCommand(joystick_izquierdo.ObtenerX());
	//servo_hombro.setVelocityCommand(joystick_izquierdo.ObtenerY());

	// Procesamiento Diferido (FSM Antirrebote)
	if (joystick_izquierdo.HayPulsacionValida()){
	    //! Registro de Evento discreto
		Logger::logINFO("Pulsador izq accionado. Conmutando LED de estado.");
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	}

	// Procesar comandos seriales entrantes si existen
	terminal.update();

	// Temporizacion por software de la comunicacion (Telemetria Periodica)
	uint32_t tiempo_actual = HAL_GetTick();
    if ((tiempo_actual - tiempo_ultimo_envio_uart) >= 1000) {
    	tiempo_ultimo_envio_uart = tiempo_actual;

    	float j1x = joystick_izquierdo.ObtenerX();
    	float j1y = joystick_izquierdo.ObtenerY();
    	float angleS1 = servo_base.getAngleDegrees();

    	// Optimización sin FPU: Separación de enteros y decimales para evitar el uso de %f
    	// Se extrae la magnitud entera (truncamiento)
    	int j1x_int = static_cast<int>(j1x);
    	// Se extrae la magnitud fraccionaria de 2 dígitos sin importar el signo original
    	int j1x_frac = static_cast<int>((j1x < 0.0f ? -j1x : j1x) * 100.0f) % 100;

    	int j1y_int = static_cast<int>(j1y);
    	int j1y_frac = static_cast<int>((j1y < 0.0f ? -j1y : j1y) * 100.0f) % 100;

    	int aS1_int = static_cast<int>(angleS1);
    	int aS1_frac = static_cast<int>((angleS1 < 0.0f ? -angleS1 : angleS1) * 100.0f) % 100;

    	// Despacho de telemetría a través del Logger optimizado utilizando %d en lugar de %f
    	// El Logger inyectará automáticamente el SysTick de hardware en el prefijo.
    	Logger::logDEBUG("RAW_X:%lu | RAW_Y:%lu | J1X:%d.%02d | J1Y:%d.%02d | S1:%d.%02d",
    			buffer_global_adc[0], buffer_global_adc[1],
				j1x_int, j1x_frac,
				j1y_int, j1y_frac,
				aS1_int, aS1_frac);
    }
}

/**
 * ===== RUTINAS DE SERVICIO DE INTERRUPCION (ISR) =====
 */

// Callback del Temporizador (Lazo cinetico de servos 50Hz)
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3){
    	servo_base.updatePosition();
    	//servo_hombro.updatePosition();
    	//servo_muneca.updatePosition();
    }
}

// Callback de las Interrupciones Externas de Pines (Botones Joystick)
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	joystick_izquierdo.NotificarInterrupcion(GPIO_Pin);
	//joystick_derecho.NotificarInterrupcion(GPIO_Pin);
}

// Callback de UART
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1){
		terminal.interrupcionRecepcionByte();
	}
}





