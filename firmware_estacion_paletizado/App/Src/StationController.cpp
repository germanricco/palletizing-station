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


// CONSTRUCTOR
Controller::Controller(TerminalSerial& terminal,
                       Joystick& joystickIzquierdo,	Joystick& joystickDerecho,
                       Servo& servoBase, Servo& servoHombro, Servo& servoCodo,
					   Actuador& efectorFinal)
    : _terminal(terminal),
      _joystickIzquierdo(joystickIzquierdo),
	  _joystickDerecho(joystickDerecho),
      _servoBase(servoBase),
	  _servoHombro(servoHombro),
	  _servoCodo(servoCodo),
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
      _chunkRequestPending(false),
	  _lastTrajectoryTick(0U) {
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

// UPDATE PRINCIPAL DEL CONTROLADOR
void Controller::update(uint32_t nowMs) {
    if (!_initialized) init();

    // Actualizar la Maquina de estados interna del actuador neumatico
    _efectorFinal.update(nowMs);

    // Transicion de succion (100%) a retencion (10%)
    if (_efectorFinal.getState() == ActuatorState::Suction) {
    	if ((nowMs - _tiempoInicioSuccion) >= 2000U ) {
    		_efectorFinal.commandHold(0.1f);
    		Logger::logDEBUG("Actuador: Vacio asegurado. Pasando a Holding");
    	}
    }

    // Lecutra de Comandos UART
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

    // Ejecutar acciones propias de cada estado
    runCurrentState();

    // Envio de Telemetria
    if ((nowMs - _tiempoUltimoEnvioTelemetria) >= 20U) {
        _tiempoUltimoEnvioTelemetria = nowMs;
        enviarTelemetria();
    }
}
// GETTERS Y AYUDAS PUBLICAS
GlobalState Controller::getState(void) const { return _state; }

GlobalState Controller::getPreviousState(void) const { return _previousState; }

const char* Controller::getStateName(void) const { return toString(_state); }

const char* Controller::stateToString(GlobalState state) { return toString(state); }

bool Controller::didStateChange(void) const { return _stateChanged; }

void Controller::clearStateChangeFlag(void) { _stateChanged = false; }

void Controller::forceFault(uint32_t nowMs) { transitionTo(GlobalState::Fault, nowMs); }

void Controller::forceEmergency(uint32_t nowMs) { transitionTo(GlobalState::Emergency, nowMs); }


// CONVERSION DE ESTADOS Y COMANDOS A TEXTO
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


// NORMALIZACIÓN Y PARSEO DE COMANDOS UART
void Controller::normalizeToUpperNoSpaces(const char* src, char* dst, uint16_t dstSize) {
	if (!src || !dst || dstSize == 0U) return;

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

    if (cmd[0] == '\0') return UartCommand::None;
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

bool Controller::tryParseGCommand(const char* rawLine, PendingAutoCommand& outCmd) {
	if (!rawLine) return false;

    // Validacion de prefijo "G1:"
	if (rawLine[0] != 'G' || rawLine[1] != '1' || rawLine[2] != ':') return false;

    // Buffer modificable local con payload
    char buffer[96] = {0};
    std::snprintf(buffer, sizeof(buffer), "%s", rawLine + 3);  // salta "G1:"

    // Separar 4 tokens
    char* tokens[4] = {nullptr};
    int tokenCount = 0;
    char* context = nullptr;
    char* token = strtok_r(buffer, ",", &context);

    while (token && tokenCount < 4) {
        tokens[tokenCount++] = token;
        token = strtok_r(nullptr, ",", &context);
    }

    // Validacion de cantidad de campos
    if (tokenCount != 4) return false;

    char* endPtr = nullptr;
    outCmd.valid = true;
    outCmd.q1 = std::strtof(tokens[0], &endPtr);
    outCmd.q2 = std::strtof(tokens[1], &endPtr);
    outCmd.q3 = std::strtof(tokens[2], &endPtr);
    outCmd.a1 = static_cast<uint8_t>(std::strtol(tokens[3], &endPtr, 10));

    return true;
}


/*
 * ================================================================
 * HELPERS FIFO PARA AUTO_RUNNING
 * ================================================================
 */
bool Controller::isAutoFifoEmpty(void) const { return (_autoFifoCount == 0U); }
bool Controller::isAutoFifoFull(void) const { return (_autoFifoCount >= AUTO_FIFO_CAPACITY); }

void Controller::clearAutoFifo(void) {
    _autoFifoHead = 0U;
    _autoFifoTail = 0U;
    _autoFifoCount = 0U;
}

void Controller::clearAutoRuntimeState(void) {
    clearAutoFifo();
    _chunkRequestPending = false;
    _lastTrajectoryTick = 0U;
}

bool Controller::enqueueAutoPoint(const PendingAutoCommand& cmd) {
	if (isAutoFifoFull()) return false;
    _autoFifo[_autoFifoTail] = cmd;
    _autoFifoTail = static_cast<uint8_t>((_autoFifoTail + 1U) % AUTO_FIFO_CAPACITY);
    _autoFifoCount++;
    return true;
}

bool Controller::dequeueAutoPoint(PendingAutoCommand& cmd) {
	if (isAutoFifoEmpty()) return false;
    cmd = _autoFifo[_autoFifoHead];
    _autoFifoHead = static_cast<uint8_t>((_autoFifoHead + 1U) % AUTO_FIFO_CAPACITY);
    _autoFifoCount--;
    return true;
}

void Controller::requestNextChunkIfNeeded(void) {
	// Low Watermark (Histeresis Inferior)
    if (_autoFifoCount >= 16) return;
    if (_chunkRequestPending) return;

    Logger::logDEBUG("AUTO_RUNNING: FIFO baja. Enviado REQ_CHUNK.");
    _terminal.enviarNormal("REQ_CHUNK\r\n");
    _chunkRequestPending = true;
}


void Controller::gestionarActuadorAuto(uint8_t a1, uint32_t nowMs) {
    if (a1 == 1) {
        if (_efectorFinal.getState() != ActuatorState::Suction &&
            _efectorFinal.getState() != ActuatorState::Holding) {

            _efectorFinal.commandSuction(1.0f);
            _tiempoInicioSuccion = nowMs; // Correccion Bug: Timeout de vacio
            Logger::logDEBUG("ACTUADOR: Iniciando Succion (Auto)");
        }
    } else {
        if (_efectorFinal.getState() != ActuatorState::Idle &&
            _efectorFinal.getState() != ActuatorState::Releasing) {

            _efectorFinal.commandRelease();
            Logger::logDEBUG("ACTUADOR: Liberando pieza (Auto)");
        }
    }
}


// CONSTRUCCIÓN DE RESPUESTAS ACK / ERR
CommandResult Controller::makeAck(const char* ackText) {
    CommandResult result{true, true, {0}};
    std::snprintf(result.response, sizeof(result.response), "%s\r\n", ackText);
    return result;
}

CommandResult Controller::makeErr(const char* errText) {
    CommandResult result{true, false, {0}};
    std::snprintf(result.response, sizeof(result.response), "%s\r\n", errText);
    return result;
}

CommandResult Controller::makeHandledNoReply(void) {
    return CommandResult{true, true, {0}};
}

CommandResult Controller::makeNotHandled(void) {
    return CommandResult{false, false, {0}};
}


// TRANSICION DE ESTADO
void Controller::transitionTo(GlobalState newState, uint32_t nowMs) {
    if (_state == newState) { return; }

    _previousState = _state;
    _state = newState;
    _stateEntryTick = nowMs;
    _stateChanged = true;

    onEnterState(_state);
}


// DESPAHO DE ENTRADA A ESTADO
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


// DESPACHO DE UPDATE SEGUN ESTADO
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


// PROCESAMIENTO PRINCIPAL DE COMANDOS UART
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


//COMANDOS LOCALES SEGÚN EL ESTADO ACTUAL
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


// FUNCIONES AUXILIARES DE LA ESTACIÓN
void Controller::detenerActuadoresManuales(void) {
    _servoBase.setTargetVelocity(0.0f);
    _servoHombro.setTargetVelocity(0.0f);
    _servoCodo.setTargetVelocity(0.0f);
}

void Controller::realizarServoInitInicial(void) {
    /* Intencionalmente vacío */
}

void Controller::realizarGoHomeNormal(void) {
    _servoBase.goHome();
    _servoHombro.goHome();
    _servoCodo.goHome();
}

void Controller::enviarTelemetria(void) {
	// Buffer de mensaje
    char msg[96];

    // Estado de Servos
    const float j1_rad = _servoBase.getAngleRadians();
    const float j2_rad = _servoHombro.getAngleRadians();
    const float j3_rad = _servoCodo.getAngleRadians();

    // Estado de Actuador
    int a1 = 0;
    ActuatorState estadoActuador = _efectorFinal.getState();
    if (estadoActuador == ActuatorState::Suction || estadoActuador == ActuatorState::Holding) {
    	a1 = 1;
    }

    std::snprintf(msg, sizeof(msg), "T:%.2f,%.2f,%.2f,%d\r\n", j1_rad, j2_rad, j3_rad, a1);
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
    _servoBase.setTargetVelocity(_joystickIzquierdo.ObtenerX());
    _servoHombro.setTargetVelocity(_joystickIzquierdo.ObtenerY());
    _servoCodo.setTargetVelocity(_joystickDerecho.ObtenerY());

    // 2. Actualizar Actuador si hay pulsación
    if (_joystickDerecho.HayPulsacionValida()) {
        Logger::logDEBUG("TELEOPERACION: pulsador derecho accionado.");
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // Feedback visual

        ActuatorState estadoActuador = _efectorFinal.getState();

        if (estadoActuador == ActuatorState::Idle || estadoActuador == ActuatorState::Releasing) {
            _efectorFinal.commandSuction(1.0f);
            _tiempoInicioSuccion = HAL_GetTick(); // Registramos el inicio
            Logger::logDEBUG("ACTUADOR: Iniciando Succion (100% PWM).");
        } else {
            _efectorFinal.commandRelease();
            Logger::logDEBUG("ACTUADOR: Liberando pieza.");
        }
    }
}


// CALLBACKS DE ENTRADA A ESTADO
void Controller::onEnterBoot(void) {
    Logger::logDEBUG("FSM -> Boot");
}

void Controller::onEnterInit(void) {
    Logger::logDEBUG("FSM -> Init");
    detenerActuadoresManuales();
    clearAutoRuntimeState();
}

void Controller::onEnterIdle(void) {
    Logger::logDEBUG("FSM -> Idle");
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
    Logger::logDEBUG("FSM -> Manual");
}

void Controller::onEnterRecording(void) {
    Logger::logDEBUG("FSM -> Recording");
    realizarGoHomeNormal();
}

void Controller::onEnterAutoReady(void) {
    Logger::logDEBUG("FSM -> AutoReady");
    detenerActuadoresManuales();
    clearAutoRuntimeState();
}

void Controller::onEnterAutoRunning(void) {
    Logger::logDEBUG("FSM -> AutoRunning");

    /*
     * Al iniciar playback desde AutoReady queremos arrancar limpio.
     * Si venimos desde AutoPaused por M114, preservamos FIFO y punto activo.
     */
    if (_previousState != GlobalState::AutoPaused) {
        clearAutoRuntimeState();
        _lastTrajectoryTick = HAL_GetTick(); // Inicia el reloj principal
        Logger::logDEBUG("AUTO_RUNNING: FIFO inicializada. Listo para solicitar 1er chunk...");
    } else {
    	_lastTrajectoryTick = HAL_GetTick(); // Resincroniza el reloj al despausar
        Logger::logDEBUG("AUTO_RUNNING: reanudando ejecucion. FIFO preservada.");
    }
}

void Controller::onEnterAutoPaused(void) {
    Logger::logDEBUG("FSM -> AutoPaused");
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


// CALLBACKS DE UPDATE POR ESTADO
void Controller::onUpdateInit(void) {}

void Controller::onUpdateIdle(void) {}

void Controller::onUpdateManual(void) { procesarTeleoperacion(); }

void Controller::onUpdateRecording(void) { procesarTeleoperacion(); }

void Controller::onUpdateAutoReady(void) {}

void Controller::onUpdateAutoRunning(void) {
    uint32_t nowMs = HAL_GetTick();

    // Metrónomo estricto a 50Hz (20ms)
    if ((nowMs - _lastTrajectoryTick) >= 20U) {

        // Sumar exactamente 20 elimina el drift acumulativo
        _lastTrajectoryTick += 20U;

        PendingAutoCommand currentPoint{};

        if (dequeueAutoPoint(currentPoint)) {
            // A. Cinemática
            float anguloGradosQ1 = currentPoint.q1 * 57.29578f;
            float anguloGradosQ2 = currentPoint.q2 * 57.29578f;
            float anguloGradosQ3 = currentPoint.q3 * 57.29578f;

            _servoBase.setDirectTargetPosition(anguloGradosQ1);
            _servoHombro.setDirectTargetPosition(anguloGradosQ2);
            _servoCodo.setDirectTargetPosition(anguloGradosQ3);

            // B. Actuador (Corregido con timestamp)
            gestionarActuadorAuto(currentPoint.a1, nowMs);
        }
        // Si hay Underflow (FIFO vacía), ignoramos silenciosamente y el brazo mantiene su posición.
    }

    // Evaluación en background
    requestNextChunkIfNeeded();
}

void Controller::onUpdateAutoPaused(void) {}

void Controller::onUpdateFault(void) {}

void Controller::onUpdateEmergency(void) {}


// CALLBACKS DE COMANDOS LOCALES
void Controller::onManualCommand(const char* rawCommand) {
    Logger::logDEBUG("CMD(MANUAL): %s", rawCommand);
}

void Controller::onRecordingCommand(const char* rawCommand) {
    Logger::logDEBUG("CMD(RECORDING): %s", rawCommand);
}

void Controller::onAutoReadyCommand(const char* rawCommand) {
    PendingAutoCommand parsedCmd{};
    if (!tryParseGCommand(rawCommand, parsedCmd)) return;
    Logger::logDEBUG("AUTO_READY: G ignorado. Primer chunk inicia despues de ACK_M112.");
}

void Controller::onAutoRunningCommand(const char* rawCommand) {
    PendingAutoCommand parsedCmd{};
    if (!tryParseGCommand(rawCommand, parsedCmd)) return;

    if (!enqueueAutoPoint(parsedCmd)) {
        Logger::logERROR("AUTO_RUNNING: FIFO llena. Punto descartado.");
        return;
    }

    // High Watermark (Histeresis superior para soltar el candado de petición)
    if (_chunkRequestPending && (_autoFifoCount >= 24)) {
    	_chunkRequestPending = false;
    }
}

void Controller::onAutoPausedCommand(const char* rawCommand) {
    Logger::logDEBUG("CMD(AUTO_PAUSED): %s", rawCommand);
}

void Controller::onIdleCommand(const char* rawCommand) {
    Logger::logDEBUG("CMD(IDLE): %s", rawCommand);
}

} // namespace Station
