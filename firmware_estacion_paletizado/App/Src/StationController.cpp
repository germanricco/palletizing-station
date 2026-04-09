#include "StationController.hpp"
#include "Logger.hpp"
#include "stm32f1xx_hal.h"

#include <cstdio>
#include <cstring>
#include <cctype>

namespace Station {

/*
 * ================================================================
 * RESUMEN DEL FLUJO GENERAL DE ESTADOS
 * ================================================================
 *
 * Arranque automático:
 *   Boot -> Init
 *
 * Init queda como estado seguro de espera.
 * Desde Init solo se acepta:
 *
 *   Init --M99--> Idle
 *
 * Transiciones por comando:
 *
 *   Idle --M100--> Manual
 *   Idle --M101--> AutoReady
 *   Idle --M98-->  Init
 *
 *   Manual --M110--> Recording
 *   Manual --M99-->  Idle
 *   Manual --M98-->  Init
 *
 *   Recording --M111--> Manual
 *   Recording --M99-->  Idle
 *
 *   AutoReady --M112--> AutoRunning
 *   AutoReady --M99-->  Idle
 *   AutoReady --M98-->  Init
 *
 *   AutoRunning --M113--> AutoPaused
 *   AutoRunning --M99-->  Idle
 *
 *   AutoPaused --M114--> AutoRunning
 *   AutoPaused --M113--> AutoReady
 *   AutoPaused --M99-->  Idle
 *
 * Prioridad absoluta:
 *   Cualquier estado --M00--> Emergency
 *
 * En estados Fault y Emergency se bloquean comandos normales.
 * ================================================================
 */

/*
 * ================================================================
 * CONSTRUCTOR
 * ================================================================
 */
Controller::Controller(TerminalSerial& terminal,
                       Joystick& joystickIzquierdo,
                       Servo& servoBase)
    : _terminal(terminal),
      _joystickIzquierdo(joystickIzquierdo),
      _servoBase(servoBase),
      _state(GlobalState::Boot),
      _previousState(GlobalState::Boot),
      _stateChanged(false),
      _initialized(false),
      _primerIngresoIdle(true),
      _stateEntryTick(0U),
      _tiempoUltimoEnvioTelemetria(0U) {
}

/*
 * ================================================================
 * INICIALIZACIÓN GENERAL
 * ================================================================
 *
 * Deja el controlador listo en estado Boot.
 * El avance Boot -> Init ocurre automáticamente durante update().
 *
 * A diferencia del comportamiento anterior, Init ya no transiciona
 * automáticamente a Idle. Ahora Init se usa como estado seguro
 * de espera y sincronización con la PC.
 */
void Controller::init(void) {
    _state = GlobalState::Boot;
    _previousState = GlobalState::Boot;
    _stateChanged = true;
    _initialized = true;
    _primerIngresoIdle = true;
    _stateEntryTick = 0U;
    _tiempoUltimoEnvioTelemetria = 0U;

    onEnterState(_state);
}

/*
 * ================================================================
 * UPDATE PRINCIPAL DEL CONTROLADOR
 * ================================================================
 *
 * Responsabilidades:
 * 1) asegurar inicialización
 * 2) procesar comandos UART pendientes
 * 3) ejecutar transiciones automáticas de arranque
 * 4) ejecutar lógica periódica del estado actual
 * 5) enviar telemetría periódica
 */
void Controller::update(uint32_t nowMs) {
    if (!_initialized) {
        init();
    }

    /*
     * ------------------------------------------------
     * 1) Procesamiento de comandos UART
     * ------------------------------------------------
     */
    if (_terminal.hayComandoPendiente()) {
        const char* comando = _terminal.obtenerComandoPendiente();

        /*
         * STATUS se responde como mensaje normal,
         * no como ACK/ERR del protocolo.
         */
        if (std::strcmp(comando, "STATUS") == 0) {
            enviarEstadoActualPorUart();
        } else {
            const CommandResult result = handleUartLine(comando, nowMs);

            /*
             * ACK / ERR se envían por la cola de alta prioridad.
             */
            if (result.handled && (result.response[0] != '\0')) {
                _terminal.enviarACK(result.response);
            }
        }

        _terminal.limpiarComandoPendiente();
    }

    /*
     * ------------------------------------------------
     * 2) Transiciones automáticas de arranque
     * ------------------------------------------------
     *
     * Solo se mantiene Boot -> Init.
     * Init ya no pasa automáticamente a Idle.
     */
    switch (_state) {
        case GlobalState::Boot:
            transitionTo(GlobalState::Init, nowMs);
            break;

        default:
            break;
    }

    /*
     * ------------------------------------------------
     * 3) Lógica periódica según estado actual
     * ------------------------------------------------
     */
    runCurrentState();

    /*
     * ------------------------------------------------
     * 4) Envío de telemetría periódica
     * ------------------------------------------------
     *
     * 20 ms entre envíos -> 50 Hz
     */
    if ((nowMs - _tiempoUltimoEnvioTelemetria) >= 20U) {
        _tiempoUltimoEnvioTelemetria = nowMs;
        enviarTelemetria();
    }
}

/*
 * ================================================================
 * GETTERS Y AYUDAS PÚBLICAS
 * ================================================================
 */
GlobalState Controller::getState(void) const {
    return _state;
}

GlobalState Controller::getPreviousState(void) const {
    return _previousState;
}

const char* Controller::getStateName(void) const {
    return toString(_state);
}

const char* Controller::stateToString(GlobalState state) {
    return toString(state);
}

bool Controller::didStateChange(void) const {
    return _stateChanged;
}

void Controller::clearStateChangeFlag(void) {
    _stateChanged = false;
}

/*
 * ================================================================
 * FORZADO DE ESTADOS CRÍTICOS
 * ================================================================
 */
void Controller::forceFault(uint32_t nowMs) {
    transitionTo(GlobalState::Fault, nowMs);
}

void Controller::forceEmergency(uint32_t nowMs) {
    transitionTo(GlobalState::Emergency, nowMs);
}

/*
 * ================================================================
 * CONVERSIÓN DE ESTADOS Y COMANDOS A TEXTO
 * ================================================================
 */
const char* Controller::toString(GlobalState state) {
    switch (state) {
        case GlobalState::Boot:        return "Boot";
        case GlobalState::Init:        return "Init";
        case GlobalState::Idle:        return "Idle";
        case GlobalState::Manual:      return "Manual";
        case GlobalState::Recording:   return "Recording";
        case GlobalState::AutoReady:   return "AutoReady";
        case GlobalState::AutoRunning: return "AutoRunning";
        case GlobalState::AutoPaused:  return "AutoPaused";
        case GlobalState::Fault:       return "Fault";
        case GlobalState::Emergency:   return "Emergency";
        default:                       return "UnknownState";
    }
}

const char* Controller::toString(UartCommand command) {
    switch (command) {
        case UartCommand::None:                   return "None";
        case UartCommand::M98_GoInit:             return "M98";
        case UartCommand::M99_GoIdle:             return "M99";
        case UartCommand::M100_ManualMode:        return "M100";
        case UartCommand::M101_AutoMode:          return "M101";
        case UartCommand::M110_StartRecording:    return "M110";
        case UartCommand::M111_StopRecording:     return "M111";
        case UartCommand::M112_StartPlayback:     return "M112";
        case UartCommand::M113_StopPausePlayback: return "M113";
        case UartCommand::M114_ResumePlayback:    return "M114";
        case UartCommand::M00_EStop:              return "M00";
        case UartCommand::Status:                 return "STATUS";
        default:                                  return "Unknown";
    }
}

/*
 * ================================================================
 * NORMALIZACIÓN Y PARSEO DE COMANDOS UART
 * ================================================================
 *
 * El parser:
 * - convierte a mayúsculas
 * - elimina espacios, tabs y fin de línea
 * - devuelve un enum
 */
void Controller::normalizeToUpperNoSpaces(const char* src, char* dst, uint16_t dstSize) {
    if ((src == nullptr) || (dst == nullptr) || (dstSize == 0U)) {
        return;
    }

    uint16_t j = 0U;

    for (uint16_t i = 0U; src[i] != '\0' && j < (dstSize - 1U); ++i) {
        const unsigned char c = static_cast<unsigned char>(src[i]);

        if ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n')) {
            continue;
        }

        dst[j++] = static_cast<char>(std::toupper(c));
    }

    dst[j] = '\0';
}

UartCommand Controller::parseUartCommand(const char* text) {
    char cmd[32] = {0};
    normalizeToUpperNoSpaces(text, cmd, sizeof(cmd));

    if (cmd[0] == '\0') {
        return UartCommand::None;
    }

    if (std::strcmp(cmd, "M98") == 0)    return UartCommand::M98_GoInit;
    if (std::strcmp(cmd, "M99") == 0)    return UartCommand::M99_GoIdle;
    if (std::strcmp(cmd, "M100") == 0)   return UartCommand::M100_ManualMode;
    if (std::strcmp(cmd, "M101") == 0)   return UartCommand::M101_AutoMode;
    if (std::strcmp(cmd, "M110") == 0)   return UartCommand::M110_StartRecording;
    if (std::strcmp(cmd, "M111") == 0)   return UartCommand::M111_StopRecording;
    if (std::strcmp(cmd, "M112") == 0)   return UartCommand::M112_StartPlayback;
    if (std::strcmp(cmd, "M113") == 0)   return UartCommand::M113_StopPausePlayback;
    if (std::strcmp(cmd, "M114") == 0)   return UartCommand::M114_ResumePlayback;
    if (std::strcmp(cmd, "M00") == 0)    return UartCommand::M00_EStop;
    if (std::strcmp(cmd, "STATUS") == 0) return UartCommand::Status;

    return UartCommand::Unknown;
}

/*
 * ================================================================
 * CONSTRUCCIÓN DE RESPUESTAS ACK / ERR
 * ================================================================
 */
CommandResult Controller::makeAck(const char* ackText) {
    CommandResult result{};
    result.handled = true;
    result.success = true;
    std::snprintf(result.response, sizeof(result.response), "%s\r\n", ackText);
    return result;
}

CommandResult Controller::makeErr(const char* errText) {
    CommandResult result{};
    result.handled = true;
    result.success = false;
    std::snprintf(result.response, sizeof(result.response), "%s\r\n", errText);
    return result;
}

CommandResult Controller::makeHandledNoReply(void) {
    CommandResult result{};
    result.handled = true;
    result.success = true;
    result.response[0] = '\0';
    return result;
}

CommandResult Controller::makeNotHandled(void) {
    CommandResult result{};
    result.handled = false;
    result.success = false;
    result.response[0] = '\0';
    return result;
}

/*
 * ================================================================
 * TRANSICIÓN DE ESTADO
 * ================================================================
 *
 * Actualiza:
 * - estado previo
 * - estado actual
 * - tick de entrada
 * - flag de cambio
 */
void Controller::transitionTo(GlobalState newState, uint32_t nowMs) {
    if (_state == newState) {
        return;
    }

    _previousState = _state;
    _state = newState;
    _stateEntryTick = nowMs;
    _stateChanged = true;

    onEnterState(_state);
}

/*
 * ================================================================
 * DESPACHO DE ENTRADA A ESTADO
 * ================================================================
 */
void Controller::onEnterState(GlobalState state) {
    switch (state) {
        case GlobalState::Boot:        onEnterBoot(); break;
        case GlobalState::Init:        onEnterInit(); break;
        case GlobalState::Idle:        onEnterIdle(); break;
        case GlobalState::Manual:      onEnterManual(); break;
        case GlobalState::Recording:   onEnterRecording(); break;
        case GlobalState::AutoReady:   onEnterAutoReady(); break;
        case GlobalState::AutoRunning: onEnterAutoRunning(); break;
        case GlobalState::AutoPaused:  onEnterAutoPaused(); break;
        case GlobalState::Fault:       onEnterFault(); break;
        case GlobalState::Emergency:   onEnterEmergency(); break;
        default: break;
    }
}

/*
 * ================================================================
 * DESPACHO DE UPDATE SEGÚN ESTADO
 * ================================================================
 */
void Controller::runCurrentState(void) {
    switch (_state) {
        case GlobalState::Init:        onUpdateInit(); break;
        case GlobalState::Idle:        onUpdateIdle(); break;
        case GlobalState::Manual:      onUpdateManual(); break;
        case GlobalState::Recording:   onUpdateRecording(); break;
        case GlobalState::AutoReady:   onUpdateAutoReady(); break;
        case GlobalState::AutoRunning: onUpdateAutoRunning(); break;
        case GlobalState::AutoPaused:  onUpdateAutoPaused(); break;
        case GlobalState::Fault:       onUpdateFault(); break;
        case GlobalState::Emergency:   onUpdateEmergency(); break;
        default: break;
    }
}

/*
 * ================================================================
 * PROCESAMIENTO PRINCIPAL DE COMANDOS UART
 * ================================================================
 */
CommandResult Controller::handleUartLine(const char* rawLine, uint32_t nowMs) {
    if (rawLine == nullptr) {
        return makeErr("ERR:NULL_CMD");
    }

    const UartCommand command = parseUartCommand(rawLine);

    /*
     * Prioridad absoluta: emergencia
     */
    if (command == UartCommand::M00_EStop) {
        transitionTo(GlobalState::Emergency, nowMs);
        return makeAck("ACK_M00");
    }

    /*
     * STATUS se responde por otro camino.
     */
    if (command == UartCommand::Status) {
        return makeHandledNoReply();
    }

    /*
     * Si no es un comando global reconocido, se intenta tratar
     * como comando local del estado actual.
     */
    if ((command == UartCommand::Unknown) || (command == UartCommand::None)) {
        processStateLocalCommand(rawLine);
        return makeNotHandled();
    }

    /*
     * En estados críticos se rechazan comandos normales.
     */
    if (_state == GlobalState::Emergency) {
        return makeErr("ERR:EMERGENCY_ACTIVE");
    }

    if (_state == GlobalState::Fault) {
        return makeErr("ERR:FAULT_ACTIVE");
    }

    /*
     * Reglas de transición según estado actual.
     */
    switch (_state) {
        case GlobalState::Init:
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::Idle:
            if (command == UartCommand::M100_ManualMode) {
                transitionTo(GlobalState::Manual, nowMs);
                return makeAck("ACK_M100");
            }
            if (command == UartCommand::M101_AutoMode) {
                transitionTo(GlobalState::AutoReady, nowMs);
                return makeAck("ACK_M101");
            }
            if (command == UartCommand::M98_GoInit) {
                transitionTo(GlobalState::Init, nowMs);
                return makeAck("ACK_M98");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::Manual:
            if (command == UartCommand::M110_StartRecording) {
                transitionTo(GlobalState::Recording, nowMs);
                return makeAck("ACK_M110");
            }
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            if (command == UartCommand::M98_GoInit) {
                transitionTo(GlobalState::Init, nowMs);
                return makeAck("ACK_M98");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::Recording:
            if (command == UartCommand::M111_StopRecording) {
                transitionTo(GlobalState::Manual, nowMs);
                return makeAck("ACK_M111");
            }
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::AutoReady:
            if (command == UartCommand::M112_StartPlayback) {
                transitionTo(GlobalState::AutoRunning, nowMs);
                return makeAck("ACK_M112");
            }
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            if (command == UartCommand::M98_GoInit) {
                transitionTo(GlobalState::Init, nowMs);
                return makeAck("ACK_M98");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::AutoRunning:
            if (command == UartCommand::M113_StopPausePlayback) {
                transitionTo(GlobalState::AutoPaused, nowMs);
                return makeAck("ACK_M113");
            }
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::AutoPaused:
            if (command == UartCommand::M114_ResumePlayback) {
                transitionTo(GlobalState::AutoRunning, nowMs);
                return makeAck("ACK_M114");
            }
            if (command == UartCommand::M113_StopPausePlayback) {
                transitionTo(GlobalState::AutoReady, nowMs);
                return makeAck("ACK_M113");
            }
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::Boot:
            return makeErr("ERR:NOT_READY");

        default:
            return makeErr("ERR:UNEXPECTED_STATE");
    }
}

/*
 * ================================================================
 * COMANDOS LOCALES SEGÚN EL ESTADO ACTUAL
 * ================================================================
 */
void Controller::processStateLocalCommand(const char* rawLine) {
    switch (_state) {
        case GlobalState::Manual:
            onManualCommand(rawLine);
            break;

        case GlobalState::Recording:
            onRecordingCommand(rawLine);
            break;

        case GlobalState::AutoReady:
            onAutoReadyCommand(rawLine);
            break;

        case GlobalState::AutoRunning:
            onAutoRunningCommand(rawLine);
            break;

        case GlobalState::AutoPaused:
            onAutoPausedCommand(rawLine);
            break;

        case GlobalState::Idle:
            onIdleCommand(rawLine);
            break;

        default:
            break;
    }
}

/*
 * ================================================================
 * FUNCIONES AUXILIARES DE LA ESTACIÓN
 * ================================================================
 */

/*
 * Detiene el movimiento manual del servo.
 */
void Controller::detenerActuadoresManuales(void) {
    _servoBase.setVelocityCommand(0.0f);
}

/*
 * Se deja por estructura y documentación.
 * La inicialización real del servo ocurre en main_cpp.cpp.
 */
void Controller::realizarServoInitInicial(void) {
    /* Intencionalmente vacío */
}

/*
 * Lleva el servo a HOME.
 */
void Controller::realizarGoHomeNormal(void) {
    _servoBase.goHome();
}

/*
 * Envía telemetría periódica por UART.
 *
 * Formato actual:
 *   T:j1,j2,j3,a1,a2
 */
void Controller::enviarTelemetria(void) {
    char msg[96];

    const float j1_rad = _servoBase.getAngleRadians();
    const float j2_rad = 0.0f;   // TODO: integrar servo 2
    const float j3_rad = 0.0f;   // TODO: integrar servo 3
    const int a1 = 0;            // TODO: integrar bomba de vacío
    const int a2 = 0;            // TODO: integrar solenoide

    std::snprintf(msg, sizeof(msg),
                  "T:%.2f,%.2f,%.2f,%d,%d\r\n",
                  j1_rad, j2_rad, j3_rad, a1, a2);

    _terminal.enviarNormal(msg);
}

/*
 * Envía el estado actual y el anterior por UART.
 */
void Controller::enviarEstadoActualPorUart(void) {
    char msg[96];

    std::snprintf(msg, sizeof(msg),
                  "EVT:STATE,%s,%s\r\n",
                  getStateName(),
                  stateToString(getPreviousState()));

    _terminal.enviarNormal(msg);
}

/*
 * ================================================================
 * CALLBACKS DE ENTRADA A ESTADO
 * ================================================================
 */
void Controller::onEnterBoot(void) {
    Logger::logINFO("FSM -> Boot");
}

void Controller::onEnterInit(void) {
    Logger::logINFO("FSM -> Init");
    detenerActuadoresManuales();

    /*
     * Init se usa como estado seguro de espera.
     * No se hace HOME automáticamente al entrar aquí.
     * Tampoco se envían mensajes extra por UART.
     */
}

void Controller::onEnterIdle(void) {
    Logger::logINFO("FSM -> Idle");
    detenerActuadoresManuales();

    /*
     * Primer ingreso a IDLE:
     * no repetimos init del servo.
     * En ingresos posteriores sí hacemos goHome().
     *
     * Nota:
     * si querés que también haga HOME en el primer ingreso a Idle
     * cuando venga desde Init, entonces habría que ajustar esta lógica.
     * Por ahora se mantiene el comportamiento previo del proyecto.
     */
    if (_primerIngresoIdle) {
        realizarServoInitInicial();
        _primerIngresoIdle = false;
    } else {
        realizarGoHomeNormal();
    }
}

void Controller::onEnterManual(void) {
    Logger::logINFO("FSM -> Manual");
}

void Controller::onEnterRecording(void) {
    Logger::logINFO("FSM -> Recording");

    /*
     * La grabación arranca llevando el sistema a HOME
     * como referencia inicial conocida.
     */
    realizarGoHomeNormal();
}

void Controller::onEnterAutoReady(void) {
    Logger::logINFO("FSM -> AutoReady");
    detenerActuadoresManuales();
}

void Controller::onEnterAutoRunning(void) {
    Logger::logINFO("FSM -> AutoRunning");
    detenerActuadoresManuales();
}

void Controller::onEnterAutoPaused(void) {
    Logger::logINFO("FSM -> AutoPaused");
    detenerActuadoresManuales();
}

void Controller::onEnterFault(void) {
    Logger::logERROR("FSM -> Fault");
    detenerActuadoresManuales();
    _terminal.enviarNormal("EVT:FAULT\r\n");
}

void Controller::onEnterEmergency(void) {
    Logger::logERROR("FSM -> Emergency");
    detenerActuadoresManuales();
    _terminal.enviarNormal("EVT:EMERGENCY\r\n");
}

/*
 * ================================================================
 * CALLBACKS DE UPDATE POR ESTADO
 * ================================================================
 */
void Controller::onUpdateInit(void) {
    /*
     * En Init el sistema queda en espera segura:
     * - sin movimiento manual
     * - sin HOME automático
     * - esperando M99 para pasar a Idle
     */
    detenerActuadoresManuales();
}

void Controller::onUpdateIdle(void) {
    detenerActuadoresManuales();
}

void Controller::onUpdateManual(void) {
    /*
     * En modo manual, el joystick controla la velocidad del servo.
     */
    _servoBase.setVelocityCommand(_joystickIzquierdo.ObtenerX());

    /*
     * Ejemplo actual:
     * si se detecta pulsación válida, se conmuta un LED.
     */
    if (_joystickIzquierdo.HayPulsacionValida()) {
        Logger::logINFO("MANUAL: pulsador izq accionado. Conmutando LED.");
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    }
}

void Controller::onUpdateRecording(void) {
    /*
     * En grabación, el operador sigue moviendo manualmente
     * el eje mientras el sistema queda listo para registrar
     * la trayectoria.
     */
    _servoBase.setVelocityCommand(_joystickIzquierdo.ObtenerX());

    /*
     * TODO:
     * Guardar puntos de trayectoria en memoria o buffer.
     */
}

void Controller::onUpdateAutoReady(void) {
    detenerActuadoresManuales();
}

void Controller::onUpdateAutoRunning(void) {
    detenerActuadoresManuales();

    /*
     * TODO:
     * Ejecutar trayectoria automática.
     */
}

void Controller::onUpdateAutoPaused(void) {
    detenerActuadoresManuales();
}

void Controller::onUpdateFault(void) {
    detenerActuadoresManuales();
}

void Controller::onUpdateEmergency(void) {
    detenerActuadoresManuales();
}

/*
 * ================================================================
 * CALLBACKS DE COMANDOS LOCALES
 * ================================================================
 */
void Controller::onManualCommand(const char* rawCommand) {
    Logger::logINFO("CMD(MANUAL): %s", rawCommand);
}

void Controller::onRecordingCommand(const char* rawCommand) {
    Logger::logINFO("CMD(RECORDING): %s", rawCommand);
}

void Controller::onAutoReadyCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_READY): %s", rawCommand);
}

void Controller::onAutoRunningCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_RUNNING): %s", rawCommand);
}

void Controller::onAutoPausedCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_PAUSED): %s", rawCommand);
}

void Controller::onIdleCommand(const char* rawCommand) {
    Logger::logINFO("CMD(IDLE): %s", rawCommand);
}

} // namespace Station
