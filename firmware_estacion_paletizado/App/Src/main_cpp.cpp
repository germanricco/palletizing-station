#include "main_cpp.hpp"
#include "Config.hpp"
#include "Joystick.hpp"
#include "Servo.hpp"
#include "MotorDC.hpp"
#include "Actuador.hpp"
#include "TerminalSerial.hpp"
#include "Logger.hpp"
#include "StationController.hpp"
#include "SensorIR.hpp"

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

Joystick joystick_derecho(&buffer_global_adc[2], &buffer_global_adc[3],
                          GPIOB, GPIO_PIN_6,
                          Config::JOYSTICK_DERECHO_X_CENTER,
                          Config::JOYSTICK_DERECHO_Y_CENTER,
                          Config::JOYSTICK_DEADZONE);

// Servos
Servo servo_base(&TIM3->CCR1,
                 Config::SERVO_BASE_MIN_ANGLE,
                 Config::SERVO_BASE_MAX_ANGLE,
                 Config::SERVO_BASE_MIN_PULSE_US,
                 Config::SERVO_BASE_MAX_PULSE_US,
                 Config::SERVO_BASE_HOME_ANGLE,
                 Config::SERVO_BASE_MAX_SPEED);

Servo servo_hombro(&TIM3->CCR2,
				 Config::SERVO_HOMBRO_MIN_ANGLE,
		         Config::SERVO_HOMBRO_MAX_ANGLE,
		         Config::SERVO_HOMBRO_MIN_PULSE_US,
		         Config::SERVO_HOMBRO_MAX_PULSE_US,
		         Config::SERVO_HOMBRO_HOME_ANGLE,
		         Config::SERVO_HOMBRO_MAX_SPEED);

Servo servo_codo(&TIM3->CCR3,
				 Config::SERVO_CODO_MIN_ANGLE,
		         Config::SERVO_CODO_MAX_ANGLE,
                 Config::SERVO_CODO_MIN_PULSE_US,
                 Config::SERVO_CODO_MAX_PULSE_US,
		         Config::SERVO_CODO_HOME_ANGLE,
		         Config::SERVO_CODO_MAX_SPEED);


// MotorDC
MotorDC motor_cinta(&TIM2->CCR3, 49, 0);

//SensorIR//IMPORTANTE: suponemos que el IR da un nivel LOW cuando detecta por eso RESET
SensorIR sensor_fin_cinta(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET,30);

// Actuador
Actuador efector_final(&TIM2->CCR1, &TIM2->CCR2, 49, 800);

// Terminal
TerminalSerial terminal(&huart1);

// Controlador
Station::Controller stationController(terminal,
									joystick_izquierdo, joystick_derecho,
									servo_base, servo_hombro, servo_codo,
									efector_final);


// ==== INICIALIZACION DEL SISTEMA ====
void Cpp_Main_Init(void) {
	// Terminal
    terminal.init();

    // Logger
    Logger::init(&terminal, Config::DEFAULT_LOG_LEVEL);
    Logger::logINFO("Inicializando sistema...");

    // ADCs
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    // Servos
    servo_base.init();
    servo_hombro.init();
    servo_codo.init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim3);

    // Actuador
    efector_final.init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // MotorDC
    motor_cinta.init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Sensor IR de fin de cinta
    sensor_fin_cinta.init(HAL_GetTick());

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
    joystick_derecho.Actualizar();

    terminal.update();

    const uint32_t tiempo_actual = HAL_GetTick();

    bool cambio_sensor_fin_cinta = sensor_fin_cinta.update(tiempo_actual);

    if (cambio_sensor_fin_cinta)
    {
        if (sensor_fin_cinta.objetoDetectado())
        {
            Logger::logINFO("SENSOR IR: Objeto detectado al final de la cinta.");
        }
        else
        {
            Logger::logINFO("SENSOR IR: Fin de cinta libre.");
        }
    }

    // ===================================================================================
    /**
     * RUTINA DE PRUEBA: PERFIL DE VELOCIDAD MOTOR CINTA
     */
    // Ciclo de prueba de 12000 ms (12 segundos)
    uint32_t tiempo_ciclo = tiempo_actual % 12000;
    float velocidad_prueba = 0.0f;

    if (tiempo_ciclo < 1000) {
    	// Fase 1: Aceleración Lineal (0 a 1 seg)
        // Mapea el tiempo de (0 a 1000 ms) al rango de velocidad (0.0f a 0.5f)
        velocidad_prueba = (static_cast<float>(tiempo_ciclo) / 1000.0f) * 0.25f;
    }
    else if (tiempo_ciclo < 11000) {
        // Fase 2: Mantenimiento de Velocidad (1 a 11 seg)
        // Mantiene la velocidad constante durante 10 segundos
        velocidad_prueba = 0.25f;
    }
    else {
        // Fase 3: Desaceleración Lineal (11 a 12 seg)
        // Mapea el tiempo de (11000 a 12000 ms) para reducir la velocidad de 0.5f a 0.0f
        velocidad_prueba = 0.25f - ((static_cast<float>(tiempo_ciclo - 11000) / 1000.0f) * 0.25f);
    }

    // Inyección de la velocidad calculada al motor
    //motor_cinta.setSpeed(velocidad_prueba);
    float velocidad_motor_cinta = velocidad_prueba;

    if (sensor_fin_cinta.objetoDetectado())
    {
        velocidad_motor_cinta = 0.0f;
    }

    motor_cinta.setSpeed(velocidad_motor_cinta);
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
        servo_hombro.update();
        servo_codo.update();
    }
}

// Interrupcion externa (evento de pulsador)
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)
    {
        joystick_izquierdo.NotificarInterrupcion(GPIO_Pin);
    }

    if (GPIO_Pin == GPIO_PIN_6)
    {
        joystick_derecho.NotificarInterrupcion(GPIO_Pin);
    }

    if (sensor_fin_cinta.pinCoincide(GPIO_Pin))
    {
        sensor_fin_cinta.notificarInterrupcion(GPIO_Pin);
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
