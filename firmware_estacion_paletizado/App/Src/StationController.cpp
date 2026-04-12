#include "StationController.hpp"
#include "Logger.hpp"
#include "stm32f1xx_hal.h"

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdlib>

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
      _tiempoUltimoEnvioTelemetria(0U),
      _pendingAutoCommand{false, 0.0f, 0.0f, 0.0f, 0U, 0U},
      _hasPendingAutoCommand(false),
      _autoFifo{},
      _autoFifoHead(0U),
      _autoFifoTail(0U),
      _autoFifoCount(0U),
      _activeAutoPoint{false, 0.0f, 0.0f, 0.0f, 0U, 0U},
      _hasActiveAutoPoint(false),
      _chunkRequestPending(false) {
}

/*
 * ================================================================
 * INICIALIZACIÓN GENERAL
 * ================================================================
 */
void Controller::init(void) {
    _state = GlobalState::Boot;
    _previousState = GlobalState::Boot;
    _stateChanged = true;
    _initialized = true;
    _primerIngresoIdle = true;
    _stateEntryTick = 0U;
    _tiempoUltimoEnvioTelemetria = 0U;
    _pendingAutoCommand = {false, 0.0f, 0.0f, 0.0f, 0U, 0U};
    _hasPendingAutoCommand = false;
    clearAutoRuntimeState();

    onEnterState(_state);
}

/*
 * ================================================================
 * UPDATE PRINCIPAL DEL CONTROLADOR
 * ================================================================
 */
void Controller::update(uint32_t nowMs) {
    if (!_initialized) {
        init();
    }

    if (_terminal.hayComandoPendiente()) {
        const char* comando = _terminal.obtenerComandoPendiente();

        if (std::strcmp(comando, "STATUS") == 0) {
            enviarEstadoActualPorUart();
        } else {
            const CommandResult result = handleUartLine(comando, nowMs);

            if (result.handled && (result.response[0] != '\0')) {
                _terminal.enviarACK(result.response);
            }
        }

        _terminal.limpiarComandoPendiente();
    }

    switch (_state) {
        case GlobalState::Boot:
            transitionTo(GlobalState::Init, nowMs);
            break;

        default:
            break;
    }

    runCurrentState();

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
 * Parser local para comando G:q1,q2,q3,a1,a2
 * ================================================================
 */
bool Controller::tryParseGCommand(const char* rawLine, PendingAutoCommand& outCmd) {
    if (rawLine == nullptr) {
        return false;
    }

    if ((rawLine[0] != 'G') || (rawLine[1] != ':')) {
        return false;
    }

    char buffer[96] = {0};
    std::snprintf(buffer, sizeof(buffer), "%s", rawLine + 2);

    char* tokens[5] = {nullptr};
    int tokenCount = 0;

    char* context = nullptr;
    char* token = strtok_r(buffer, ",", &context);

    while ((token != nullptr) && (tokenCount < 5)) {
        tokens[tokenCount++] = token;
        token = strtok_r(nullptr, ",", &context);
    }

    if (tokenCount != 5) {
        return false;
    }

    /*
     * Se aceptan silenciosamente campos extra.
     * Si el emisor manda un sexto campo accidentalmente,
     * la implementación actual lo ignora a propósito.
     */

    char* endPtr = nullptr;

    endPtr = nullptr;
    const float q1 = std::strtof(tokens[0], &endPtr);
    if ((endPtr == tokens[0]) || (*endPtr != '\0')) {
        return false;
    }

    endPtr = nullptr;
    const float q2 = std::strtof(tokens[1], &endPtr);
    if ((endPtr == tokens[1]) || (*endPtr != '\0')) {
        return false;
    }

    endPtr = nullptr;
    const float q3 = std::strtof(tokens[2], &endPtr);
    if ((endPtr == tokens[2]) || (*endPtr != '\0')) {
        return false;
    }

    endPtr = nullptr;
    const long a1Long = std::strtol(tokens[3], &endPtr, 10);
    if ((endPtr == tokens[3]) || (*endPtr != '\0')) {
        return false;
    }
    if (!((a1Long == 0) || (a1Long == 1))) {
        return false;
    }

    endPtr = nullptr;
    const long a2Long = std::strtol(tokens[4], &endPtr, 10);
    if ((endPtr == tokens[4]) || (*endPtr != '\0')) {
        return false;
    }
    if (!((a2Long == 0) || (a2Long == 1))) {
        return false;
    }

    outCmd.valid = true;
    outCmd.q1 = q1;
    outCmd.q2 = q2;
    outCmd.q3 = q3;
    outCmd.a1 = static_cast<uint8_t>(a1Long);
    outCmd.a2 = static_cast<uint8_t>(a2Long);

    return true;
}

/*
 * ================================================================
 * HELPERS FIFO PARA AUTO_RUNNING
 * ================================================================
 */
bool Controller::isAutoFifoEmpty(void) const {
    return (_autoFifoCount == 0U);
}

bool Controller::isAutoFifoFull(void) const {
    return (_autoFifoCount >= AUTO_FIFO_CAPACITY);
}

void Controller::clearAutoFifo(void) {
    _autoFifoHead = 0U;
    _autoFifoTail = 0U;
    _autoFifoCount = 0U;
}

void Controller::clearAutoRuntimeState(void) {
    clearAutoFifo();
    _activeAutoPoint = {false, 0.0f, 0.0f, 0.0f, 0U, 0U};
    _hasActiveAutoPoint = false;
    _chunkRequestPending = false;
}

bool Controller::enqueueAutoPoint(const PendingAutoCommand& cmd) {
    if (isAutoFifoFull()) {
        return false;
    }

    _autoFifo[_autoFifoTail] = cmd;
    _autoFifoTail = static_cast<uint8_t>((_autoFifoTail + 1U) % AUTO_FIFO_CAPACITY);
    _autoFifoCount++;
    return true;
}

bool Controller::dequeueAutoPoint(PendingAutoCommand& cmd) {
    if (isAutoFifoEmpty()) {
        return false;
    }

    cmd = _autoFifo[_autoFifoHead];
    _autoFifoHead = static_cast<uint8_t>((_autoFifoHead + 1U) % AUTO_FIFO_CAPACITY);
    _autoFifoCount--;
    return true;
}

void Controller::startNextAutoPointIfNeeded(void) {
    if (_hasActiveAutoPoint) {
        return;
    }

    PendingAutoCommand nextPoint{};
    if (!dequeueAutoPoint(nextPoint)) {
        return;
    }

    _activeAutoPoint = nextPoint;
    _hasActiveAutoPoint = true;

    Logger::logINFO(
        "AUTO_RUNNING: nuevo punto activo q1=%.3f q2=%.3f q3=%.3f a1=%u a2=%u | fifo_restante=%u",
        _activeAutoPoint.q1,
        _activeAutoPoint.q2,
        _activeAutoPoint.q3,
        static_cast<unsigned>(_activeAutoPoint.a1),
        static_cast<unsigned>(_activeAutoPoint.a2),
        static_cast<unsigned>(_autoFifoCount)
    );
}

void Controller::processActiveAutoPoint(void) {
    if (!_hasActiveAutoPoint) {
        return;
    }

    /*
     * PRIMERA ETAPA:
     * consumo lógico inmediato para validar el protocolo de streaming.
     * Más adelante acá se conectará _activeAutoPoint.q1 al servo real.
     */
    Logger::logINFO(
        "AUTO_RUNNING: punto consumido q1=%.3f q2=%.3f q3=%.3f a1=%u a2=%u",
        _activeAutoPoint.q1,
        _activeAutoPoint.q2,
        _activeAutoPoint.q3,
        static_cast<unsigned>(_activeAutoPoint.a1),
        static_cast<unsigned>(_activeAutoPoint.a2)
    );

    _hasActiveAutoPoint = false;
}

void Controller::requestNextChunkIfNeeded(void) {
    if (_hasActiveAutoPoint) {
        return;
    }

    if (!isAutoFifoEmpty()) {
        return;
    }

    if (_chunkRequestPending) {
        return;
    }

    _terminal.enviarNormal("REQ_CHUNK\r\n");
    _chunkRequestPending = true;

    Logger::logINFO("AUTO_RUNNING: FIFO vacia. Enviado REQ_CHUNK.");
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

    if (command == UartCommand::M00_EStop) {
        transitionTo(GlobalState::Emergency, nowMs);
        return makeAck("ACK_M00");
    }

    if (command == UartCommand::Status) {
        return makeHandledNoReply();
    }

    if ((command == UartCommand::Unknown) || (command == UartCommand::None)) {
        processStateLocalCommand(rawLine);
        return makeNotHandled();
    }

    if (_state == GlobalState::Emergency) {
        return makeErr("ERR:EMERGENCY_ACTIVE");
    }

    if (_state == GlobalState::Fault) {
        return makeErr("ERR:FAULT_ACTIVE");
    }

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
void Controller::detenerActuadoresManuales(void) {
    _servoBase.setVelocityCommand(0.0f);
}

void Controller::realizarServoInitInicial(void) {
    /* Intencionalmente vacío */
}

void Controller::realizarGoHomeNormal(void) {
    _servoBase.goHome();
}

void Controller::enviarTelemetria(void) {
    char msg[96];

    const float j1_rad = _servoBase.getAngleRadians();
    const float j2_rad = 1.20f;   // TODO: integrar servo 2
    const float j3_rad = -2.00f;  // TODO: integrar servo 3
    const int a1 = 0;             // TODO: integrar bomba de vacío
    const int a2 = 0;             // TODO: integrar solenoide

    std::snprintf(msg, sizeof(msg),
                  "T:%.2f,%.2f,%.2f,%d,%d\r\n",
                  j1_rad, j2_rad, j3_rad, a1, a2);

    _terminal.enviarNormal(msg);
}

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
    _pendingAutoCommand = {false, 0.0f, 0.0f, 0.0f, 0U, 0U};
    _hasPendingAutoCommand = false;
    clearAutoRuntimeState();
}

void Controller::onEnterIdle(void) {
    Logger::logINFO("FSM -> Idle");
    detenerActuadoresManuales();
    _pendingAutoCommand = {false, 0.0f, 0.0f, 0.0f, 0U, 0U};
    _hasPendingAutoCommand = false;
    clearAutoRuntimeState();

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
    realizarGoHomeNormal();
}

void Controller::onEnterAutoReady(void) {
    Logger::logINFO("FSM -> AutoReady");
    detenerActuadoresManuales();
    _pendingAutoCommand = {false, 0.0f, 0.0f, 0.0f, 0U, 0U};
    _hasPendingAutoCommand = false;
    clearAutoRuntimeState();
}

void Controller::onEnterAutoRunning(void) {
    Logger::logINFO("FSM -> AutoRunning");
    detenerActuadoresManuales();

    /*
     * Al iniciar playback desde AutoReady queremos arrancar limpio.
     * Si venimos desde AutoPaused por M114, preservamos FIFO y punto activo.
     */
    if (_previousState != GlobalState::AutoPaused) {
        clearAutoRuntimeState();
        Logger::logINFO("AUTO_RUNNING: FIFO inicializada. Esperando puntos G...");
    } else {
        Logger::logINFO("AUTO_RUNNING: reanudando ejecucion. FIFO preservada.");
    }
}

void Controller::onEnterAutoPaused(void) {
    Logger::logINFO("FSM -> AutoPaused");
    detenerActuadoresManuales();
}

void Controller::onEnterFault(void) {
    Logger::logERROR("FSM -> Fault");
    detenerActuadoresManuales();
    clearAutoRuntimeState();
    _terminal.enviarNormal("EVT:FAULT\r\n");
}

void Controller::onEnterEmergency(void) {
    Logger::logERROR("FSM -> Emergency");
    detenerActuadoresManuales();
    clearAutoRuntimeState();
    _terminal.enviarNormal("EVT:EMERGENCY\r\n");
}

/*
 * ================================================================
 * CALLBACKS DE UPDATE POR ESTADO
 * ================================================================
 */
void Controller::onUpdateInit(void) {
    detenerActuadoresManuales();
}

void Controller::onUpdateIdle(void) {
    detenerActuadoresManuales();
}

void Controller::onUpdateManual(void) {
    _servoBase.setVelocityCommand(_joystickIzquierdo.ObtenerX());

    if (_joystickIzquierdo.HayPulsacionValida()) {
        Logger::logINFO("MANUAL: pulsador izq accionado. Conmutando LED.");
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    }
}

void Controller::onUpdateRecording(void) {
    _servoBase.setVelocityCommand(_joystickIzquierdo.ObtenerX());
}

void Controller::onUpdateAutoReady(void) {
    detenerActuadoresManuales();
}

void Controller::onUpdateAutoRunning(void) {
    detenerActuadoresManuales();

    startNextAutoPointIfNeeded();
    processActiveAutoPoint();
    requestNextChunkIfNeeded();
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

    PendingAutoCommand parsedCmd{};

    if (!tryParseGCommand(rawCommand, parsedCmd)) {
        Logger::logINFO("AUTO_READY: comando local ignorado o formato G invalido.");
        return;
    }

    _pendingAutoCommand = parsedCmd;
    _hasPendingAutoCommand = true;

    Logger::logINFO(
        "AUTO_READY: G almacenado q1=%.3f q2=%.3f q3=%.3f a1=%u a2=%u",
        _pendingAutoCommand.q1,
        _pendingAutoCommand.q2,
        _pendingAutoCommand.q3,
        static_cast<unsigned>(_pendingAutoCommand.a1),
        static_cast<unsigned>(_pendingAutoCommand.a2)
    );
}

void Controller::onAutoRunningCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_RUNNING): %s", rawCommand);

    PendingAutoCommand parsedCmd{};

    if (!tryParseGCommand(rawCommand, parsedCmd)) {
        Logger::logINFO("AUTO_RUNNING: comando local ignorado o formato G invalido.");
        return;
    }

    if (!enqueueAutoPoint(parsedCmd)) {
        Logger::logERROR("AUTO_RUNNING: FIFO llena. Punto descartado.");
        return;
    }

    _chunkRequestPending = false;

    Logger::logINFO(
        "AUTO_RUNNING: punto encolado q1=%.3f q2=%.3f q3=%.3f a1=%u a2=%u | fifo_count=%u",
        parsedCmd.q1,
        parsedCmd.q2,
        parsedCmd.q3,
        static_cast<unsigned>(parsedCmd.a1),
        static_cast<unsigned>(parsedCmd.a2),
        static_cast<unsigned>(_autoFifoCount)
    );
}

void Controller::onAutoPausedCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_PAUSED): %s", rawCommand);
}

void Controller::onIdleCommand(const char* rawCommand) {
    Logger::logINFO("CMD(IDLE): %s", rawCommand);
}

} // namespace Station
