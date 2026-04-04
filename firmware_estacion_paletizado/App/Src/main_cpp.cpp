#include "main_cpp.hpp"
#include "Config.hpp"
#include "Joystick.hpp"
#include "Servo.hpp"
#include "TerminalSerial.hpp"
#include "Logger.hpp"
#include "StationFSM.hpp"

#include "stm32f1xx_hal.h"
#include <cstdio>
#include <cstring>

extern "C" {
    extern ADC_HandleTypeDef hadc1;
    extern TIM_HandleTypeDef htim3;
    extern UART_HandleTypeDef huart1;
}

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
Station::FSM stationFsm;

static uint32_t tiempo_ultimo_envio_telemetria = 0U;
static bool g_primerIngresoIdle = true;

/*
 * ================================================================
 * HELPERS LOCALES
 * ================================================================
 */

static void detenerActuadoresManuales(void) {
    servo_base.setVelocityCommand(0.0f);
}

/*
 * Primer ingreso a IDLE tras el arranque.
 * OJO:
 * servo_base.init() ya se hace en Cpp_Main_Init().
 * Acá NO lo repetimos para evitar doble inicialización.
 */
static void realizarServoInitInicial(void) {
    // Intencionalmente vacío.
    // Se deja la función para conservar la estructura actual del código.
}

/*
 * Ingresos posteriores a IDLE.
 */
static void realizarGoHomeNormal(void) {
    servo_base.goHome();
}

static void enviarTelemetria(void) {
    char msg[96];

    const float j1_rad = servo_base.getAngleRadians();
    const float j2_rad = 1.571f;   // TODO: integrar servo 2
    const float j3_rad = -1.571f;   // TODO: integrar servo 3
    const int a1 = 0;            // TODO: integrar bomba de vacío
    const int a2 = 0;            // TODO: integrar solenoide

    std::snprintf(msg, sizeof(msg),
                  "T:%.2f,%.2f,%.2f,%d,%d\r\n",
                  j1_rad, j2_rad, j3_rad, a1, a2);

    terminal.enviarNormal(msg);
}

static void enviarEstadoActualPorUart(void) {
    char msg[96];
    std::snprintf(msg, sizeof(msg),
                  "EVT:STATE,%s,%s\r\n",
                  stationFsm.getStateName(),
                  Station::toString(stationFsm.getPreviousState()));

    terminal.enviarNormal(msg);
}

/*
 * ================================================================
 * CALLBACKS DE ENTRADA A ESTADO
 * ================================================================
 */

static void onEnterBoot(void) {
    Logger::logINFO("FSM -> Boot");
}

static void onEnterInit(void) {
    Logger::logINFO("FSM -> Init");
}

static void onEnterIdle(void) {
    Logger::logINFO("FSM -> Idle");
    detenerActuadoresManuales();

    if (g_primerIngresoIdle) {
        realizarServoInitInicial();
        g_primerIngresoIdle = false;
    } else {
        realizarGoHomeNormal();
    }
}

static void onEnterManual(void) {
    Logger::logINFO("FSM -> Manual");
}

static void onEnterRecording(void) {
    Logger::logINFO("FSM -> Recording");

    /*
     * La grabación debe comenzar en HOME.
     */
    realizarGoHomeNormal();
}

static void onEnterAutoReady(void) {
    Logger::logINFO("FSM -> AutoReady");
    detenerActuadoresManuales();
}

static void onEnterAutoRunning(void) {
    Logger::logINFO("FSM -> AutoRunning");
    detenerActuadoresManuales();

    /*
     * Si luego quieren obligar a arrancar playback desde HOME,
     * pueden descomentar esto.
     */
    // realizarGoHomeNormal();
}

static void onEnterAutoPaused(void) {
    Logger::logINFO("FSM -> AutoPaused");
    detenerActuadoresManuales();
}

static void onEnterFault(void) {
    Logger::logERROR("FSM -> Fault");
    detenerActuadoresManuales();

    /*
     * Evento importante, pero no ACK.
     * Va por prioridad normal.
     */
    terminal.enviarNormal("EVT:FAULT\r\n");
}

static void onEnterEmergency(void) {
    Logger::logERROR("FSM -> Emergency");
    detenerActuadoresManuales();

    /*
     * TODO:
     * Desenergizar realmente motores/actuadores.
     */
    terminal.enviarNormal("EVT:EMERGENCY\r\n");
}

/*
 * ================================================================
 * CALLBACKS DE UPDATE POR ESTADO
 * ================================================================
 */

static void onUpdateIdle(void) {
    detenerActuadoresManuales();
}

static void onUpdateManual(void) {
    servo_base.setVelocityCommand(joystick_izquierdo.ObtenerX());

    if (joystick_izquierdo.HayPulsacionValida()) {
        Logger::logINFO("MANUAL: pulsador izq accionado. Conmutando LED.");
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    }
}

static void onUpdateRecording(void) {
    /*
     * En RECORDING se sigue moviendo manualmente mientras se registra
     * la trayectoria por telemetría y/o buffer local.
     */
    servo_base.setVelocityCommand(joystick_izquierdo.ObtenerX());

    /*
     * TODO:
     * Guardar puntos de trayectoria en memoria si corresponde.
     */
}

static void onUpdateAutoReady(void) {
    detenerActuadoresManuales();

    /*
     * TODO:
     * Validar trayectoria cargada, sensores, condiciones seguras, etc.
     */
}

static void onUpdateAutoRunning(void) {
    detenerActuadoresManuales();

    /*
     * TODO:
     * Ejecutar trayectoria automática.
     */
}

static void onUpdateAutoPaused(void) {
    detenerActuadoresManuales();

    /*
     * TODO:
     * Mantener índice/punto actual para reanudar.
     */
}

static void onUpdateFault(void) {
    detenerActuadoresManuales();
}

static void onUpdateEmergency(void) {
    detenerActuadoresManuales();
}

/*
 * ================================================================
 * CALLBACKS DE COMANDOS LOCALES
 * ================================================================
 */

static void onManualCommand(const char* rawCommand) {
    Logger::logINFO("CMD(MANUAL): %s", rawCommand);
}

static void onRecordingCommand(const char* rawCommand) {
    Logger::logINFO("CMD(RECORDING): %s", rawCommand);
}

static void onAutoReadyCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_READY): %s", rawCommand);
}

static void onAutoRunningCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_RUNNING): %s", rawCommand);
}

static void onAutoPausedCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_PAUSED): %s", rawCommand);
}

static void onIdleCommand(const char* rawCommand) {
    Logger::logINFO("CMD(IDLE): %s", rawCommand);
}

/*
 * ================================================================
 * CONFIGURACIÓN DE HOOKS DE LA FSM
 * ================================================================
 */

static void configurarHooksFSM(void) {
    Station::Hooks hooks{};

    hooks.onEnterBoot        = onEnterBoot;
    hooks.onEnterInit        = onEnterInit;
    hooks.onEnterIdle        = onEnterIdle;
    hooks.onEnterManual      = onEnterManual;
    hooks.onEnterRecording   = onEnterRecording;
    hooks.onEnterAutoReady   = onEnterAutoReady;
    hooks.onEnterAutoRunning = onEnterAutoRunning;
    hooks.onEnterAutoPaused  = onEnterAutoPaused;
    hooks.onEnterFault       = onEnterFault;
    hooks.onEnterEmergency   = onEnterEmergency;

    hooks.onUpdateIdle        = onUpdateIdle;
    hooks.onUpdateManual      = onUpdateManual;
    hooks.onUpdateRecording   = onUpdateRecording;
    hooks.onUpdateAutoReady   = onUpdateAutoReady;
    hooks.onUpdateAutoRunning = onUpdateAutoRunning;
    hooks.onUpdateAutoPaused  = onUpdateAutoPaused;
    hooks.onUpdateFault       = onUpdateFault;
    hooks.onUpdateEmergency   = onUpdateEmergency;

    hooks.onManualCommand      = onManualCommand;
    hooks.onRecordingCommand   = onRecordingCommand;
    hooks.onAutoReadyCommand   = onAutoReadyCommand;
    hooks.onAutoRunningCommand = onAutoRunningCommand;
    hooks.onAutoPausedCommand  = onAutoPausedCommand;
    hooks.onIdleCommand        = onIdleCommand;

    stationFsm.setHooks(hooks);
}

/*
 * ================================================================
 * API C EXPUESTA A main.c
 * ================================================================
 */

void Cpp_Main_Init(void) {
    terminal.init();

    Logger::init(&terminal, Config::DEFAULT_LOG_LEVEL);
    Logger::logINFO("Inicializando sistema...");

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    /*
     * Única inicialización real del servo.
     */
    servo_base.init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);

    configurarHooksFSM();
    stationFsm.init();

    tiempo_ultimo_envio_telemetria = HAL_GetTick();

    terminal.enviarNormal(
        "PROTO: M100 M101 M110 M111 M112 M113 M114 M99 M00 STATUS\r\n");
}

void Cpp_Main_Loop(void) {
    joystick_izquierdo.Actualizar();
    terminal.update();

    const uint32_t tiempo_actual = HAL_GetTick();

    if (terminal.hayComandoPendiente()) {
        const char* comando = terminal.obtenerComandoPendiente();

        /*
         * STATUS se responde acá como mensaje normal.
         */
        if (std::strcmp(comando, "STATUS") == 0) {
            enviarEstadoActualPorUart();
        } else {
            const Station::CommandResult result =
                stationFsm.handleUartLine(comando, tiempo_actual);

            /*
             * Las respuestas del protocolo (ACK/ERR) salen con prioridad alta.
             */
            if (result.handled && (result.response[0] != '\0')) {
                terminal.enviarACK(result.response);
            }
        }

        terminal.limpiarComandoPendiente();
    }

    stationFsm.update(tiempo_actual);

    if (stationFsm.didStateChange()) {
        Logger::logDEBUG("Cambio de estado detectado: actual=%s | previo=%s",
                         stationFsm.getStateName(),
                         Station::toString(stationFsm.getPreviousState()));
        stationFsm.clearStateChangeFlag();
    }

    /*
     * Se mantiene telemetría a 50 Hz como en tu versión actual.
     * Sale con prioridad normal.
     */
    if ((tiempo_actual - tiempo_ultimo_envio_telemetria) >= 20U) {
        tiempo_ultimo_envio_telemetria = tiempo_actual;
        enviarTelemetria();
    }
}

/*
 * ================================================================
 * ISR / CALLBACKS HAL
 * ================================================================
 */

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        servo_base.updatePosition();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    joystick_izquierdo.NotificarInterrupcion(GPIO_Pin);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.interrupcionRecepcionByte();
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxCompleteISR();
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxErrorISR();
    }
}
