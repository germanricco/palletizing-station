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
    extern TIM_HandleTypeDef htim3;      // Timer del lazo cinemático (50 Hz)
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

// Marca temporal para telemetría periódica.
static uint32_t tiempo_ultimo_envio_uart = 0U;

/*
 * ================================================================
 * HELPERS LOCALES
 * ================================================================
 * Estas funciones encapsulan acciones mínimas por estado.
 * No implementan todavía trayectorias ni FIFO: solamente dejan la
 * estructura lista para que ustedes la completen después.
 */

static void detenerActuadoresManuales(void) {
    // Actualemnte el único actuador integrado en el repo es servo_base.
    // Para anular el control manual alcanza con mandarle velocidad 0.
    servo_base.setVelocityCommand(0.0f);
}

static bool esComandoTexto(const char* recibido, const char* esperado) {
    if ((recibido == nullptr) || (esperado == nullptr)) {
        return false;
    }
    return (std::strcmp(recibido, esperado) == 0);
}

static void enviarEstadoActualPorUart(void) {
    char msg[96];
    std::snprintf(msg, sizeof(msg),
                  "FSM STATE: %s | PREV: %s\r\n",
                  stationFsm.getStateName(),
                  Station::toString(stationFsm.getPreviousState()));
    terminal.enviarMensaje(msg);
}

/*
 * ================================================================
 * CALLBACKS DE ENTRADA A ESTADO (Hooks de Station::FSM)
 * ================================================================
 * Acá se ejecuta una sola vez cada vez que la FSM entra a un estado.
 * Esto sirve para loguear transiciones y dejar acciones de setup.
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
}

static void onEnterManual(void) {
    Logger::logINFO("FSM -> Manual");
}

static void onEnterAuto(void) {
    Logger::logINFO("FSM -> Auto");
    detenerActuadoresManuales();

    /*
     * Base conceptual:
     * acá más adelante ustedes pueden inicializar el ejecutor FIFO,
     * limpiar una cola de trayectorias, cargar una rutina guardada, etc.
     */
}

static void onEnterPaused(void) {
    Logger::logINFO("FSM -> Paused");
    detenerActuadoresManuales();
}

static void onEnterFault(void) {
    Logger::logERROR("FSM -> Fault");
    detenerActuadoresManuales();
}

static void onEnterEmergencyStop(void) {
    Logger::logERROR("FSM -> EmergencyStop");
    detenerActuadoresManuales();
}

static void onEnterHoming(void) {
    Logger::logINFO("FSM -> Homing");

    /*
     * Por ahora dejo una acción mínima y segura: mandar el servo a HOME.
     * Más adelante ustedes pueden reemplazar esto por una rutina de homing
     * real con sensores, varios ejes o trayectoria específica.
     */
    servo_base.goHome();
}

/*
 * ================================================================
 * CALLBACKS DE UPDATE POR ESTADO
 * ================================================================
 * Se ejecutan en cada vuelta del loop mientras la FSM permanece en el
 * estado correspondiente.
 */

static void onUpdateIdle(void) {
    // Estado de espera. No se mueve nada por defecto.
    detenerActuadoresManuales();
}

static void onUpdateManual(void) {
    /*
     * Este es el comportamiento manual base:
     * el joystick izquierdo sigue mandando la velocidad del servo base,
     * igual que en tu implementación original, pero ahora SOLO cuando
     * la FSM está realmente en Manual.
     */
    servo_base.setVelocityCommand(joystick_izquierdo.ObtenerX());

    // Se conserva también el ejemplo del pulsador del joystick.
    if (joystick_izquierdo.HayPulsacionValida()) {
        Logger::logINFO("MANUAL: pulsador izq accionado. Conmutando LED.");
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    }
}

static void onUpdateAuto(void) {
    /*
     * Placeholder de automático:
     * por ahora no ejecuta trayectorias. La idea es que acá, más adelante,
     * llamen a algo del estilo:
     *
     *   trajectoryExecutor.update();
     *
     * o al módulo que consuma la FIFO.
     */
    detenerActuadoresManuales();
}

static void onUpdatePaused(void) {
    detenerActuadoresManuales();
}

static void onUpdateFault(void) {
    detenerActuadoresManuales();
}

static void onUpdateEmergencyStop(void) {
    detenerActuadoresManuales();
}

static void onUpdateHoming(void) {
    /*
     * Base mínima:
     * el movimiento a home se hace una vez en onEnterHoming().
     * Acá no hacemos nada más por ahora.
     *
     * Cuando implementen homing real, este update puede preguntar sensores,
     * tiempos, posiciones alcanzadas, etc.
     */
    detenerActuadoresManuales();
}

/*
 * ================================================================
 * CALLBACKS DE COMANDOS LOCALES POR ESTADO
 * ================================================================
 * Estos se invocan cuando llega por UART un texto que NO es un comando
 * global de modo (MANUAL, AUTO, STOP, PAUSE, ...), sino un comando propio
 * del estado actual.
 *
 * Ejemplos futuros:
 * - En Manual: MOVE BASE 120, GRIP OPEN, SAVEPOINT
 * - En Auto:   LOAD TRAJ 1, RUN FIFO, ABORT STEP
 */

static void onManualCommand(const char* rawCommand) {
    Logger::logINFO("CMD(MANUAL): %s", rawCommand);

    /*
     * Acá después ustedes pueden despachar funciones concretas, por ejemplo:
     *
     * if (strncmp(rawCommand, "MOVE ", 5) == 0) { mover(...); }
     * if (strcmp(rawCommand, "SAVEPOINT") == 0) { guardarPunto(); }
     * if (strcmp(rawCommand, "GRIP OPEN") == 0) { actuador(...); }
     */
}

static void onAutoCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO): %s", rawCommand);

    /*
     * Acá después van sus comandos del modo automático, por ejemplo:
     * - cargar trayectoria guardada
     * - iniciar FIFO
     * - pausar una secuencia interna
     * - abortar una trayectoria
     */
}

static void onIdleCommand(const char* rawCommand) {
    Logger::logINFO("CMD(IDLE): %s", rawCommand);

    /*
     * En Idle normalmente no movería actuadores.
     * Pero sí puede ser útil aceptar comandos de consulta, configuración o
     * preparación antes de pasar a Manual o Auto.
     */
}

/*
 * ================================================================
 * CONFIGURACIÓN DE HOOKS DE LA FSM
 * ================================================================
 */

static void configurarHooksFSM(void) {
    Station::Hooks hooks{};

    hooks.onEnterBoot          = onEnterBoot;
    hooks.onEnterInit          = onEnterInit;
    hooks.onEnterIdle          = onEnterIdle;
    hooks.onEnterManual        = onEnterManual;
    hooks.onEnterAuto          = onEnterAuto;
    hooks.onEnterPaused        = onEnterPaused;
    hooks.onEnterFault         = onEnterFault;
    hooks.onEnterEmergencyStop = onEnterEmergencyStop;
    hooks.onEnterHoming        = onEnterHoming;

    hooks.onUpdateIdle          = onUpdateIdle;
    hooks.onUpdateManual        = onUpdateManual;
    hooks.onUpdateAuto          = onUpdateAuto;
    hooks.onUpdatePaused        = onUpdatePaused;
    hooks.onUpdateFault         = onUpdateFault;
    hooks.onUpdateEmergencyStop = onUpdateEmergencyStop;
    hooks.onUpdateHoming        = onUpdateHoming;

    hooks.onManualCommand = onManualCommand;
    hooks.onAutoCommand   = onAutoCommand;
    hooks.onIdleCommand   = onIdleCommand;

    stationFsm.setHooks(hooks);
}

/*
 * ================================================================
 * API C EXPUESTA A main.c
 * ================================================================
 */

void Cpp_Main_Init(void) {
    // 1) Inicializa la recepción UART no bloqueante.
    terminal.init();

    // 2) Inicializa el logger sobre la misma terminal UART.
    Logger::init(&terminal, Config::DEFAULT_LOG_LEVEL);
    Logger::logINFO("Inicializando sistema...");

    // 3) Arranca el ADC por DMA para leer ambos ejes del joystick.
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    // 4) Inicializa el servo existente en el repo.
    servo_base.init();

    // 5) Arranca PWM y temporizador base del mismo TIM3.
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);

    // 6) Configura hooks y luego inicializa la FSM.
    configurarHooksFSM();
    stationFsm.init();

    tiempo_ultimo_envio_uart = HAL_GetTick();

    // Mensaje de ayuda mínimo para probar desde terminal.
    terminal.enviarMensaje(
        "Comandos FSM: MANUAL | AUTO | HOME | STOP | PAUSE | RESUME | RESET | ESTOP | STATUS\r\n");
}

void Cpp_Main_Loop(void) {
    // -----------------------------------------------------------------
    // A) Mantener actualizadas las entradas analógicas del joystick.
    //    Esto se hace siempre, independientemente del estado de la FSM.
    // -----------------------------------------------------------------
    joystick_izquierdo.Actualizar();

    // -----------------------------------------------------------------
    // B) Dar servicio a la UART.
    //    TerminalSerial arma una línea completa y la deja pendiente.
    // -----------------------------------------------------------------
    terminal.update();

    if (terminal.hayComandoPendiente()) {
        const char* comando = terminal.obtenerComandoPendiente();

        /*
         * STATUS se responde desde acá porque tu StationFSM actual lo reconoce,
         * pero no despacha ningún callback específico para ese caso.
         */
        if (esComandoTexto(comando, "STATUS")) {
            enviarEstadoActualPorUart();
        }

        // La línea completa se entrega a la FSM para que haga la transición o
        // despache el comando local según el estado.
        stationFsm.handleUartLine(comando);

        // Una vez consumido el comando, se limpia el buffer y se rearma la UART.
        terminal.limpiarComandoPendiente();
    }

    // -----------------------------------------------------------------
    // C) Actualizar la FSM.
    //    IMPORTANTE: la FSM decide qué callback de estado ejecutar.
    // -----------------------------------------------------------------
    const uint32_t tiempo_actual = HAL_GetTick();
    stationFsm.update(tiempo_actual);

    // Si querés, esto permite observar visualmente cuándo hubo transición.
    if (stationFsm.didStateChange()) {
        Logger::logDEBUG("Cambio de estado detectado: actual=%s | previo=%s",
                         stationFsm.getStateName(),
                         Station::toString(stationFsm.getPreviousState()));
        stationFsm.clearStateChangeFlag();
    }

    // -----------------------------------------------------------------
    // D) Telemetría periódica.
    //    Se conserva tu filosofía original, agregando el nombre del estado.
    // -----------------------------------------------------------------
    if ((tiempo_actual - tiempo_ultimo_envio_uart) >= 1000U) {
        tiempo_ultimo_envio_uart = tiempo_actual;

        const float j1x = joystick_izquierdo.ObtenerX();
        const float j1y = joystick_izquierdo.ObtenerY();
        const float angleS1 = servo_base.getAngleDegrees();

        const int j1x_int  = static_cast<int>(j1x);
        const int j1x_frac = static_cast<int>((j1x < 0.0f ? -j1x : j1x) * 100.0f) % 100;

        const int j1y_int  = static_cast<int>(j1y);
        const int j1y_frac = static_cast<int>((j1y < 0.0f ? -j1y : j1y) * 100.0f) % 100;

        const int aS1_int  = static_cast<int>(angleS1);
        const int aS1_frac = static_cast<int>((angleS1 < 0.0f ? -angleS1 : angleS1) * 100.0f) % 100;

        Logger::logDEBUG("STATE:%s | RAW_X:%lu | RAW_Y:%lu | J1X:%d.%02d | J1Y:%d.%02d | S1:%d.%02d",
                         stationFsm.getStateName(),
                         buffer_global_adc[0],
                         buffer_global_adc[1],
                         j1x_int, j1x_frac,
                         j1y_int, j1y_frac,
                         aS1_int, aS1_frac);
    }
}

/*
 * ================================================================
 * ISR / CALLBACKS HAL
 * ================================================================
 */

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // Se mantiene exactamente la misma filosofía original:
        // el servo integra la posición en la ISR periódica.
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
