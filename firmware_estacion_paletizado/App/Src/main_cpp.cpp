#include "main_cpp.hpp"
#include "Config.hpp"
#include "Joystick.hpp"
#include "Servo.hpp"
#include "TerminalSerial.hpp"
#include "Logger.hpp"
#include "StationController.hpp"

#include "stm32f1xx_hal.h"
#include <cstdio>
#include <cstring>

/*
 * ================================================================
 * HANDLES HAL DEFINIDOS EN main.c
 * ================================================================
 *
 * Estos handles son creados del lado C por CubeMX / HAL y se
 * declaran acá como extern para poder usarlos desde C++.
 */
extern "C" {
    extern ADC_HandleTypeDef hadc1;
    extern TIM_HandleTypeDef htim3;
    extern UART_HandleTypeDef huart1;
}

/*
 * ================================================================
 * RECURSOS GLOBALES DEL SISTEMA
 * ================================================================
 */

/*
 * Buffer global del ADC.
 * Se utiliza para muestrear continuamente las entradas analógicas
 * asociadas al joystick mediante DMA.
 */
volatile uint32_t buffer_global_adc[4];

/*
 * Joystick izquierdo.
 *
 * Recibe:
 * - punteros a dos canales del buffer ADC
 * - puerto/pin del pulsador
 * - centros calibrados en X e Y
 * - deadzone configurada
 */
Joystick joystick_izquierdo(&buffer_global_adc[0], &buffer_global_adc[1],
                            GPIOB, GPIO_PIN_5,
                            Config::JOYSTICK_IZQUIERDO_X_CENTER,
                            Config::JOYSTICK_IZQUIERDO_Y_CENTER,
                            Config::JOYSTICK_DEADZONE);

/*
 * Servo principal del sistema.
 *
 * Trabaja sobre el compare register CCR1 del TIM3.
 * Se configuran:
 * - ángulo mínimo
 * - ángulo máximo
 * - posición HOME
 * - velocidad máxima
 */
Servo servo_base(&TIM3->CCR1,
                 Config::SERVO_BASE_MIN_ANGLE,
                 Config::SERVO_BASE_MAX_ANGLE,
                 Config::SERVO_BASE_HOME_ANGLE,
                 Config::SERVO_BASE_MAX_SPEED);

/*
 * Terminal serie principal del sistema, vinculada a USART1.
 */
TerminalSerial terminal(&huart1);

/*
 * Controlador principal de la estación.
 *
 * Este objeto concentra:
 * - FSM
 * - parseo de comandos UART
 * - telemetría
 * - lógica de estados
 */
Station::Controller stationController(terminal, joystick_izquierdo, servo_base);

/*
 * ================================================================
 * API C EXPUESTA A main.c
 * ================================================================
 */

/*
 * Inicialización principal del sistema C++.
 * Debe ejecutarse una sola vez al arranque.
 */
void Cpp_Main_Init(void) {
    /*
     * Inicialización de la terminal serie.
     */
    terminal.init();

    /*
     * Inicialización del logger.
     */
    Logger::init(&terminal, Config::DEFAULT_LOG_LEVEL);
    Logger::logINFO("Inicializando sistema...");

    /*
     * Inicio del ADC por DMA.
     *
     * El joystick leerá sus canales desde buffer_global_adc.
     */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);

    /*
     * Se deshabilitan interrupciones DMA no necesarias
     * para este esquema actual.
     */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    /*
     * Inicialización real del servo.
     */
    servo_base.init();

    /*
     * Arranque del PWM del servo y del timer base asociado.
     */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);

    /*
     * Inicialización del controlador principal.
     */
    stationController.init();

    /*
     * Mensaje inicial informativo.
     */
    terminal.enviarNormal(
        "PROTO: M100 M101 M110 M111 M112 M113 M114 M99 M00 STATUS\r\n");
}

/*
 * Loop principal del sistema.
 * Debe ejecutarse continuamente desde el while(1) del main.
 */
void Cpp_Main_Loop(void) {
    /*
     * Actualización de entradas y terminal.
     */
    joystick_izquierdo.Actualizar();
    terminal.update();

    const uint32_t tiempo_actual = HAL_GetTick();

    /*
     * Toda la lógica de alto nivel vive en stationController.
     */
    stationController.update(tiempo_actual);

    /*
     * Log opcional para detectar cambios de estado.
     */
    if (stationController.didStateChange()) {
        Logger::logDEBUG("Cambio de estado detectado: actual=%s | previo=%s",
                         stationController.getStateName(),
                         Station::Controller::stateToString(
                             stationController.getPreviousState()));

        stationController.clearStateChangeFlag();
    }
}

/*
 * ================================================================
 * CALLBACKS HAL / ISR
 * ================================================================
 */

/*
 * Callback de fin de período del timer.
 *
 * Se usa para actualizar la posición interna del servo.
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        servo_base.updatePosition();
    }
}

/*
 * Callback de interrupción externa por GPIO.
 *
 * Se utiliza para notificar al joystick eventos sobre su pulsador.
 * Se filtra explícitamente el pin del pulsador para evitar
 * procesar EXTI ajenas al joystick.
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_5) {
        joystick_izquierdo.NotificarInterrupcion(GPIO_Pin);
    }
}

/*
 * Callback de recepción UART completa.
 *
 * Notifica a la terminal serie que llegó un nuevo byte.
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.interrupcionRecepcionByte();
    }
}

/*
 * Callback de transmisión UART completa.
 *
 * Informa a la terminal que terminó una transmisión DMA.
 */
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxCompleteISR();
    }
}

/*
 * Callback de error UART.
 *
 * Se hace recuperación tanto de TX como de RX, ya que HAL puede
 * entrar aquí por distintos tipos de error del periférico UART.
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxErrorISR();
        terminal.onRxErrorISR();
    }
}
