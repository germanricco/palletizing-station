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
 *   AutoRunning --M113--> Fin de datos desde Python; espera vaciar FIFO y vuelve a AutoReady
 *   AutoRunning --M114--> AutoPaused
 *   AutoRunning --M99-->  Idle
 *
 *   AutoPaused --M115--> AutoRunning
 *   AutoPaused --M113--> AutoReady
 *   AutoPaused --M99-->  Idle
 *
 * Prioridad absoluta:
 *   Cualquier estado --M00--> Emergency
 *
 * En estados Fault y Emergency se bloquean comandos normales.
 * ================================================================
 */


// CONSTRUCTOR
Controller::Controller(TerminalSerial& terminal,
                       Joystick& joystickIzquierdo,
                       Servo& servoBase,
					   Actuador& efectorFinal)
    : _terminal(terminal),
      _joystickIzquierdo(joystickIzquierdo),
      _servoBase(servoBase),
	  _efectorFinal(efectorFinal),
      _state(GlobalState::Boot),
      _previousState(GlobalState::Boot),
      _stateChanged(false),
      _initialized(false),
      _primerIngresoIdle(true),
      _stateEntryTick(0U),
      _tiempoUltimoEnvioTelemetria(0U),
	  _tiempoInicioSuccion(0U),
      _autoFifo{},
      _autoFifoHead(0U),
      _autoFifoTail(0U),
      _autoFifoCount(0U),
      _activeAutoPoint{false, 0.00f, 0.00f, 0.00f, 0},
      _hasActiveAutoPoint(false),
      _chunkRequestPending(false),
      _waitingFirstChunkAfterStart(false),
      _playbackEofReceived(false),
      _pointsReceivedForCurrentRequest(0U),
      _autoPointCommandInjected(false),
      _autoPointStartTick(0U){
}


// INICIALIZACION
void Controller::init(void) {
    _state = GlobalState::Boot;
    _previousState = GlobalState::Boot;
    _stateChanged = true;
    _initialized = true;
    _primerIngresoIdle = true;
    _stateEntryTick = 0U;
    _tiempoUltimoEnvioTelemetria = 0U;
    clearAutoRuntimeState();

    onEnterState(_state);
}

/*
 * ================================================================
 * UPDATE PRINCIPAL DEL CONTROLADOR
 * ================================================================
 */
void Controller::update(uint32_t nowMs) {
    if (!_initialized) init();

    // Actualizar la Maquina de estados interna del actuador neumatico
    _efectorFinal.update(nowMs);

    // Transicion de succion (100%) a retencion (10%)
    if (_efectorFinal.getState() == ActuatorState::Suction) {
    	if ((nowMs - _tiempoInicioSuccion) >= 2000U ) {
    		_efectorFinal.commandHold(0.1f);
    		Logger::logINFO("Actuador: Vacio asegurado. Pasando a Holding");
    	}
    }

    // Lectura de comandos UART.
    // Durante streaming pueden llegar muchas líneas G1 seguidas; por eso drenamos
    // varios comandos por ciclo en vez de procesar solo uno.
    constexpr uint8_t MAX_COMANDOS_UART_POR_UPDATE = 12U;
    uint8_t comandosProcesados = 0U;

    while (_terminal.hayComandoPendiente() &&
           comandosProcesados < MAX_COMANDOS_UART_POR_UPDATE) {
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
        comandosProcesados++;
    }

    switch (_state) {
        case GlobalState::Boot:
            transitionTo(GlobalState::Init, nowMs);
            break;

        default:
            break;
    }

    // Ejecutar acciones propias de cada estado
    runCurrentState();

    // Envio de Telemetria
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
        case UartCommand::M113_EndPlayback:       return "M113";
        case UartCommand::M114_PausePlayback:     return "M114";
        case UartCommand::M115_ResumePlayback:    return "M115";
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
    if (std::strcmp(cmd, "M113") == 0)   return UartCommand::M113_EndPlayback;
    if (std::strcmp(cmd, "M114") == 0)   return UartCommand::M114_PausePlayback;
    if (std::strcmp(cmd, "M115") == 0)   return UartCommand::M115_ResumePlayback;
    if (std::strcmp(cmd, "M00") == 0)    return UartCommand::M00_EStop;
    if (std::strcmp(cmd, "STATUS") == 0) return UartCommand::Status;

    return UartCommand::Unknown;
}

/*
 * ================================================================
 * Parser local para comando G:q1,q2,q3
 * ================================================================
 */
bool Controller::tryParseGCommand(const char* rawLine, PendingAutoCommand& outCmd) {
    if (rawLine == nullptr) {
        return false;
    }

    // Validacion de prefijo "G1:"
    if ((rawLine[0] != 'G') || (rawLine[1] != '1') || (rawLine[2] != ':')) {
        return false;
    }

    // Buffer modificable local con payload
    char buffer[96] = {0};
    std::snprintf(buffer, sizeof(buffer), "%s", rawLine + 3);  // salta "G1:"

    // Separar 4 tokens
    char* tokens[4] = {nullptr};
    int tokenCount = 0;
    char* context = nullptr;
    char* token = strtok_r(buffer, ",", &context);

    while ((token != nullptr) && (tokenCount < 4)) {
        tokens[tokenCount++] = token;
        token = strtok_r(nullptr, ",", &context);
    }

    // Validacion de cantidad de campos
    if (tokenCount != 4) {
        return false;
    }

    char* endPtr = nullptr;
    const float q1 = std::strtof(tokens[0], &endPtr);
    const float q2 = std::strtof(tokens[1], &endPtr);
    const float q3 = std::strtof(tokens[2], &endPtr);
    const uint8_t a1 = static_cast<uint8_t>(std::strtol(tokens[3], &endPtr, 10)); // Actuador

    outCmd.valid = true;
    outCmd.q1 = q1;
    outCmd.q2 = q2;
    outCmd.q3 = q3;
    outCmd.a1 = a1;

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
    _activeAutoPoint = {false, 0.00f, 0.00f, 0.00f, 0};
    _hasActiveAutoPoint = false;
    _chunkRequestPending = false;
    _waitingFirstChunkAfterStart = false;
    _playbackEofReceived = false;
    _pointsReceivedForCurrentRequest = 0U;
    _autoPointCommandInjected = false;
    _autoPointStartTick = 0U;
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
}

void Controller::processActiveAutoPoint(void) {
    if (!_hasActiveAutoPoint) {
        return;
    }

    const uint32_t nowMs = HAL_GetTick();

    if (!_autoPointCommandInjected) {
        const float anguloGradosQ1 = _activeAutoPoint.q1 * 57.29578f;

        _servoBase.commandDirectPosition(anguloGradosQ1);
        // TODO: integrar servo hombro y servo codo cuando estén disponibles.
        // _servoHombro.commandDirectPosition(_activeAutoPoint.q2 * 57.29578f);
        // _servoCodo.commandDirectPosition(_activeAutoPoint.q3 * 57.29578f);

        if (_activeAutoPoint.a1 == 1U) {
            if (_efectorFinal.getState() != ActuatorState::Suction &&
                _efectorFinal.getState() != ActuatorState::Holding) {
                _efectorFinal.commandSuction(1.0f);
                _tiempoInicioSuccion = nowMs;
            }
        } else {
            if (_efectorFinal.getState() != ActuatorState::Idle &&
                _efectorFinal.getState() != ActuatorState::Releasing) {
                _efectorFinal.commandRelease();
            }
        }

        _autoPointCommandInjected = true;
        _autoPointStartTick = nowMs;
    }

    if ((nowMs - _autoPointStartTick) >= 20U) {
        _hasActiveAutoPoint = false;
        _autoPointCommandInjected = false;
    }
}

void Controller::requestNextChunkIfNeeded(void) {
    if (_playbackEofReceived) {
        return;
    }

    if (_chunkRequestPending) {
        return;
    }

    const uint8_t freeSlots = static_cast<uint8_t>(AUTO_FIFO_CAPACITY - _autoFifoCount);

    if (freeSlots < AUTO_CHUNK_SIZE) {
        return;
    }

    char msg[32];
    std::snprintf(msg, sizeof(msg), "REQ_CHUNK:%u\r\n", AUTO_CHUNK_SIZE);

    // REQ_CHUNK forma parte del protocolo crítico de streaming, por eso va con prioridad HIGH.
    if (_terminal.enviarACK(msg)) {
        _chunkRequestPending = true;
        _pointsReceivedForCurrentRequest = 0U;
        Logger::logINFO("AUTO_RUNNING: solicitado chunk de %u puntos.", AUTO_CHUNK_SIZE);
    } else {
        Logger::logERROR("AUTO_RUNNING: no se pudo enviar REQ_CHUNK.");
    }
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
            if (command == UartCommand::M113_EndPlayback) {
                // Python avisa que ya no enviará más puntos.
                // No respondemos ACK_M113 todavía: esperamos consumir la FIFO completa.
                _playbackEofReceived = true;
                _chunkRequestPending = false;
                _pointsReceivedForCurrentRequest = 0U;
                Logger::logINFO("AUTO_RUNNING: EOF recibido desde Python. Esperando vaciar FIFO.");
                return makeHandledNoReply();
            }
            if (command == UartCommand::M114_PausePlayback) {
                transitionTo(GlobalState::AutoPaused, nowMs);
                return makeAck("ACK_M114");
            }
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            return makeErr("ERR:INVALID_STATE");

        case GlobalState::AutoPaused:
            if (command == UartCommand::M115_ResumePlayback) {
                transitionTo(GlobalState::AutoRunning, nowMs);
                return makeAck("ACK_M115");
            }
            if (command == UartCommand::M113_EndPlayback) {
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
    _servoBase.commandVelocity(0.0f);
}

void Controller::realizarServoInitInicial(void) {
    /* Intencionalmente vacío */
}

void Controller::realizarGoHomeNormal(void) {
    _servoBase.goHome();
}

void Controller::enviarTelemetria(void) {
	// Buffer de mensaje
    char msg[96];

    // Estado de Servos
    const float j1_rad = _servoBase.getAngleRadians();
    const float j2_rad = 1.20f;   // TODO: integrar servo 2
    const float j3_rad = -2.00f;  // TODO: integrar servo 3

    // Estado de Actuador
    int a1 = 0;
    ActuatorState estadoActuador = _efectorFinal.getState();
    if (estadoActuador == ActuatorState::Suction || estadoActuador == ActuatorState::Holding) {
    	a1 = 1;
    }

    // Armar el mensaje
    std::snprintf(msg, sizeof(msg),
                  "T:%.2f,%.2f,%.2f,%d\r\n",
                  j1_rad, j2_rad, j3_rad, a1);

    // Enviarlo
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


void Controller::procesarTeleoperacion(void) {
    // 1. Actualizar Motores pasando comando de velocidad
    _servoBase.commandVelocity(_joystickIzquierdo.ObtenerX());

    // 2. Actualizar Actuador si hay pulsación
    if (_joystickIzquierdo.HayPulsacionValida()) {
        Logger::logINFO("TELEOPERACION: pulsador izq accionado. Conmutando LED y Actuador.");
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // Feedback visual

        ActuatorState estadoActuador = _efectorFinal.getState();

        if (estadoActuador == ActuatorState::Idle || estadoActuador == ActuatorState::Releasing) {
            _efectorFinal.commandSuction(1.0f);
            _tiempoInicioSuccion = HAL_GetTick(); // Registramos el inicio
            Logger::logINFO("ACTUADOR: Iniciando Succion (100% PWM).");
        } else {
            _efectorFinal.commandRelease();
            Logger::logINFO("ACTUADOR: Liberando pieza.");
        }
    }
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
    clearAutoRuntimeState();
}

void Controller::onEnterIdle(void) {
    Logger::logINFO("FSM -> Idle");
    detenerActuadoresManuales();
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
    clearAutoRuntimeState();
}

void Controller::onEnterAutoRunning(void) {
    Logger::logINFO("FSM -> AutoRunning");

    if (_previousState != GlobalState::AutoPaused) {
        clearAutoRuntimeState();
        Logger::logINFO("AUTO_RUNNING: listo para solicitar primer chunk.");
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

    // Cortar energia a actuador final
    _efectorFinal.commandIdle();

    clearAutoRuntimeState();

	// Enviar evento externo
    _terminal.enviarNormal("EVT:EMERGENCY\r\n");
}

/*
 * ================================================================
 * CALLBACKS DE UPDATE POR ESTADO
 * ================================================================
 */
void Controller::onUpdateInit(void) {}

void Controller::onUpdateIdle(void) {}

void Controller::onUpdateManual(void) {
	procesarTeleoperacion();
}

void Controller::onUpdateRecording(void) {
	procesarTeleoperacion();
}

void Controller::onUpdateAutoReady(void) {}

void Controller::onUpdateAutoRunning(void) {
    startNextAutoPointIfNeeded();
    processActiveAutoPoint();

    if (_playbackEofReceived && isAutoFifoEmpty() && !_hasActiveAutoPoint) {
        Logger::logINFO("AUTO_RUNNING: trayectoria finalizada. FIFO vacia.");
        transitionTo(GlobalState::AutoReady, HAL_GetTick());
        _terminal.enviarACK("ACK_M113\r\n");
        return;
    }

    requestNextChunkIfNeeded();
}

void Controller::onUpdateAutoPaused(void) {}

void Controller::onUpdateFault(void) {}

void Controller::onUpdateEmergency(void) {}

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

    Logger::logINFO(
        "AUTO_READY: G ignorado. El primer chunk debe llegar despues de ACK_M112 y ya en AUTO_RUNNING."
    );
}

void Controller::onAutoRunningCommand(const char* rawCommand) {
    PendingAutoCommand parsedCmd{};

    if (!tryParseGCommand(rawCommand, parsedCmd)) {
        return;
    }

    if (!enqueueAutoPoint(parsedCmd)) {
        Logger::logERROR("AUTO_RUNNING: FIFO llena. Punto descartado.");
        _terminal.enviarACK("ERR:FIFO_FULL\r\n");
        return;
    }

    if (_chunkRequestPending) {
        _pointsReceivedForCurrentRequest++;

        if (_pointsReceivedForCurrentRequest >= AUTO_CHUNK_SIZE) {
            _chunkRequestPending = false;
            _pointsReceivedForCurrentRequest = 0U;
        }
    }
}

void Controller::onAutoPausedCommand(const char* rawCommand) {
    Logger::logINFO("CMD(AUTO_PAUSED): %s", rawCommand);
}

void Controller::onIdleCommand(const char* rawCommand) {
    Logger::logINFO("CMD(IDLE): %s", rawCommand);
}

} // namespace Station
