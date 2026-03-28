#include "main_cpp.hpp"
#include "Joystick.hpp"
#include "Servo.hpp"
//#include "MotorDC.hpp"
//#include "GeneradorTrayectoria.hpp"
#include "TerminalSerial.hpp"

#include "stm32f1xx_hal.h"
#include <cstdio>
#include <cstring>

// Importación de las estructuras de hardware generadas por STM32CubeMX en main.c
extern "C" {
    extern ADC_HandleTypeDef hadc1;
    extern TIM_HandleTypeDef htim3;			// Dedicado al lazo cinetico (50Hz)
    //extern TIM_HandleTypeDef htim4;
    extern UART_HandleTypeDef huart1;
}

// Arreglo global llenado por DMA
volatile uint32_t buffer_global_adc[4];

/**
 * =====  INSTANCIACION DE OBJETOS  =====
 */
// buffer[], port, pin, ceros reales
Joystick joystick_izquierdo(&buffer_global_adc[0], &buffer_global_adc[1],
		GPIOB, GPIO_PIN_5,
		1920.0f, 1960.0f, 0.05f);
// ===================== AGREGAR METODOS O MACRO PARA CONFIGURACION DE JOYSTICKS ========================

/**
 * Joystick joystick_derecho(&buffer_global_adc[2], &buffer_global_adc[3], GPIOB, GPIO_PIN_6)
*/


// ccr1, minP, maxP, homeP, maxSpeed, minSpeed
Servo servo_base(&TIM3->CCR1, 800, 2200, 1500, 0.0f, 40.0f);
Servo servo_hombro(&TIM3->CCR2, 800, 2200, 1500, 0.0f, 40.0f);

TerminalSerial terminal(&huart1);


/**
 * =====  BANDERAS Y VARIABLES GLOBALES =====
 */

// JOYSTICKS
volatile uint8_t bandera_boton = 0;
volatile uint16_t pin_interrupcion = 0;

// Variable global para la temporizacion por software
uint32_t tiempo_ultimo_envio_uart = 0;



void Cpp_Main_Init(void) {
	// Inicia el escaneo DMA de ADC1 para los 2 canales simultaneamente
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);

	//? Inyeccion de bajo nivel para apagar interrupciones fantasmas del DMA
	__HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

	// SERVOS
	servo_base.init();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	servo_hombro.init();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // Inicia el temporizador TIM3 habilitando sus interrupciones de hardware
    HAL_TIM_Base_Start_IT(&htim3);

    // Inicia interrupcion de recepcion de la terminal
    terminal.init();

    // Inicializa el temporizador global
    tiempo_ultimo_envio_uart = HAL_GetTick();
}



void Cpp_Main_Loop(void) {
	// Actualizar estado interno de Joysticks
	joystick_izquierdo.Actualizar();

	// Transferir la intencion al actuador (-1.0 a 1.0)
	servo_base.setVelocityCommand(joystick_izquierdo.ObtenerX());

	// Recepcion de interrupcion externa por pulsadores
	if (bandera_boton == 1) {
		bandera_boton = 0;
	    joystick_izquierdo.NotificarInterrupcion(pin_interrupcion);
	    //joystick_derecho.NotificarInterrupcion(pin_interrupcion);
	}

	// Procesamiento Diferido (Antirrebote)
	if (joystick_izquierdo.HayPulsacionValida()){
	    // Encender-Apagar Led
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	}

	// Procesar comandos seriales entrantes si existen
	terminal.update();

	// Temporizacion por software de la comunicacion
	uint32_t tiempo_actual = HAL_GetTick();
    if ((tiempo_actual - tiempo_ultimo_envio_uart) >= 1000) {
    	tiempo_ultimo_envio_uart = tiempo_actual;

    	float j1x = joystick_izquierdo.ObtenerX();
    	float j1y = joystick_izquierdo.ObtenerY();
    	float angleS1 = servo_base.getAngleDegrees();

    	char mensaje_local[100];
    	sprintf(mensaje_local, "RAW_X:%lu | RAW_Y:%lu | J1X:%.2f | J1Y:%.2f | S1:%.2f\r\n",
    	        buffer_global_adc[0], buffer_global_adc[1], j1x, j1y, angleS1);

    	terminal.enviarMensaje(mensaje_local);
    }
}

/**
 * ===== RUTINAS DE SERVICIO DE INTERRUPCION (ISR) =====
 */

// Callback del Temporizador (Lazo cinetico de servos)
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3){
    	servo_base.updatePosition();
    	servo_hombro.updatePosition();
    }
}

// Callback de las Interrupciones Externas de Pines (Boton Joystick)
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Veo que pin activo la interrupcion (estan multiplexados)
	pin_interrupcion = GPIO_Pin;
	bandera_boton = 1;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1){
		terminal.interrupcionRecepcionByte();
	}
}





