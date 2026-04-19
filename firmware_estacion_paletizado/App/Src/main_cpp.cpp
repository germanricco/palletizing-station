#include "main_cpp.hpp"
#include "Config.hpp"
#include "Joystick.hpp"
#include "Servo.hpp"
#include "MotorDC.hpp"
#include "TerminalSerial.hpp"
#include "Logger.hpp"
#include "StationController.hpp"

#include "stm32f1xx_hal.h"
#include <cstdio>
#include <cstring>

extern "C" {
    extern ADC_HandleTypeDef hadc1;		// Lectura de Analogicos
    extern TIM_HandleTypeDef htim2;		// Timer para control de motorDC
    extern TIM_HandleTypeDef htim3;		// Timer para control de servos
    extern UART_HandleTypeDef huart1;	// Comunicacion UART
}


// ==== COMPONENTES DEL SISTEMA ====
// Buffer de ADC
volatile uint32_t buffer_global_adc[4];

// Joystick
Joystick joystick_izquierdo(&buffer_global_adc[0], &buffer_global_adc[1],
                            GPIOB, GPIO_PIN_5,
                            Config::JOYSTICK_IZQUIERDO_X_CENTER,
                            Config::JOYSTICK_IZQUIERDO_Y_CENTER,
                            Config::JOYSTICK_DEADZONE);

// Servo
Servo servo_base(&TIM3->CCR1,
                 Config::SERVO_BASE_MIN_ANGLE,
                 Config::SERVO_BASE_MAX_ANGLE,
                 Config::SERVO_BASE_HOME_ANGLE,
                 Config::SERVO_BASE_MAX_SPEED);

// MotorDC
MotorDC motor_cinta(&TIM2->CCR3, 49, 0);

// Terminal
TerminalSerial terminal(&huart1);

// Controlador
Station::Controller stationController(terminal, joystick_izquierdo, servo_base);


// ==== INICIALIZACION DEL SISTEMA ====
void Cpp_Main_Init(void) {
	// Terminal
    terminal.init();

    // Logger
    Logger::init(&terminal, Config::DEFAULT_LOG_LEVEL);
    Logger::logINFO("Inicializando sistema...");

    // ADC
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    // Servo
    servo_base.init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);

    // MotorDC
    motor_cinta.init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Controlador
    stationController.init();

    // Mensaje Inicial
    terminal.enviarNormal(
        "PROTO: M100 M101 M110 M111 M112 M113 M114 M99 M00 STATUS\r\n");
}


// ==== LOOP PRINCIPAL DEL SISTEMA ====
void Cpp_Main_Loop(void) {
	// Actualizacion de entradas
    joystick_izquierdo.Actualizar();
    terminal.update();

    const uint32_t tiempo_actual = HAL_GetTick();

    /**
     * RUTINA DE PRUEBA: PERFIL DE VELOCIDAD MOTOR CINTA
     */
    // Ciclo de prueba de 12000 ms (12 segundos)
    uint32_t tiempo_ciclo = tiempo_actual % 12000;
    float velocidad_prueba = 0.0f;

    if (tiempo_ciclo < 2000) {
    	// Fase 1: Reposo (0 a 2 seg)
        velocidad_prueba = 0.0f;
    }
    else if (tiempo_ciclo < 5000) {
        // Fase 2: Aceleración Lineal (2 a 5 seg)
        // Mapea el tiempo de (2000 a 5000) al rango de velocidad (0.0 a 1.0)
        velocidad_prueba = static_cast<float>(tiempo_ciclo - 2000) / 3000.0f;
    }
    else if (tiempo_ciclo < 8000) {
        // Fase 3: Mantenimiento Velocidad Máxima (5 a 8 seg)
        velocidad_prueba = 1.0f;
    }
    else if (tiempo_ciclo < 11000) {
        // Fase 4: Desaceleración Lineal (8 a 11 seg)
        // Mapea el tiempo de (8000 a 11000) al rango de velocidad (1.0 a 0.0)
        velocidad_prueba = 1.0f - (static_cast<float>(tiempo_ciclo - 8000) / 3000.0f);
    }
    else {
        // Fase 5: Reposo de estabilización (11 a 12 seg)
        velocidad_prueba = 0.0f;
    }

    // Inyección de la velocidad calculada al motor
    motor_cinta.setSpeed(velocidad_prueba);
    // ===================================================================================

    stationController.update(tiempo_actual);

    // Log para detectar cambios de estado (opcional)
    if (stationController.didStateChange()) {
        Logger::logDEBUG("Cambio de estado detectado: actual=%s | previo=%s",
                         stationController.getStateName(),
                         Station::Controller::stateToString(
                             stationController.getPreviousState()));

        stationController.clearStateChangeFlag();
    }
}

// ==== CALLBACKS ====

// Actualizacion de Servos
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        servo_base.update();
    }
}

// Interrupcion externa (evento de pulsador)
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_5) {
        joystick_izquierdo.NotificarInterrupcion(GPIO_Pin);
    }
}

// Recepcion de UART completa
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.interrupcionRecepcionByte();
    }
}

// Transmision de UART completa
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxCompleteISR();
    }
}

// Error en UART
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxErrorISR();
        terminal.onRxErrorISR();
    }
}
