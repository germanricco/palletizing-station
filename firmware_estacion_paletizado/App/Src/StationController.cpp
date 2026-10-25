#include "StationController.hpp"
#include "Logger.hpp"
#include "Config.hpp"
#include "stm32f1xx_hal.h"

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdlib>
#include <cmath>

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
 *   Manual --M110 [robot en Home]--> Recording   (si no: ERR:NOT_AT_HOME)
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
 *   AutoRunning --M113--> AutoPaused          (pausa real, iniciada por el usuario)
 *   AutoRunning --M116--> AutoRunning         (marca _endOfTrajectoryPending; no cambia de estado)
 *   AutoRunning --(FIFO vacia y EOF pendiente)--> AutoReady   (finalizacion automatica, ver onUpdateAutoRunning)
 *   AutoRunning --M99-->  Idle
 *
 *   AutoPaused --M114--> AutoRunning
 *   AutoPaused --M113--> AutoReady
 *   AutoPaused --M99-->  Idle
 *
 *   AutoRunning --(desvio de la secuencia IR/efector)--> PickRecovery   (EVT:PICK_FAIL,<causa>,<n>)
 *   PickRecovery --(en Home + caja frente al IR + reintentos disponibles)--> AutoReady
 *                                                       (EVT:PICK_RETRY,<n> + EVT:SENSOR)
 *   PickRecovery --(sin caja o sin reintentos)--> PickAlarm             (EVT:PICK_ALARM,<causa>)
 *   PickRecovery --M99--> Idle
 *   PickAlarm --M115--> AutoReady   (continuar: re-pick si hay caja; si no, la cinta avanza)
 *   PickAlarm --M113--> AutoReady   (cancelar: termina el ciclo y desarma la cinta)
 *   PickAlarm --M99-->  Idle
 *
 * Prioridad absoluta:
 *   Cualquier estado --M00--> Emergency
 *
 * En estados Fault y Emergency se bloquean comandos normales, con una unica excepcion:
 *
 *   Fault --M999--> Init
 *   Emergency --M999--> Init
 *
 * M999 = salida controlada (ver IEC 60204-1 / ISO 13850: un reset nunca debe por si solo
 * reanudar movimiento). Requiere una ventana minima de permanencia (M999_MIN_DWELL_MS) y
 * siempre aterriza en Init (pasivo), nunca en Idle (que homea al entrar) ni en un modo
 * operativo -- el operador debe re-navegar la FSM explicitamente (M99, luego modo) para
 * retomar. Fault hoy tiene un disparador real: timeout de FIFO en AutoRunning sin M116
 * (ver AUTO_FIFO_STARVATION_TIMEOUT_MS en onUpdateAutoRunning), ademas de forceFault()
 * disponible para futuros disparadores.
 *
 * HOMING BAJO DEMANDA (sub-estado interno, no un GlobalState):
 *   Disponible en Manual y Recording (donde corre procesarTeleoperacion()). El pulsador
 *   izquierdo del joystick (libre en teleoperacion) dispara goHome() en los 3 servos y
 *   BLOQUEA por completo la teleoperacion por joystick (ejes y pulsador de actuador) hasta que
 *   los 3 servos reporten isTargetReached() -- sin cancelacion por movimiento del joystick,
 *   porque presionar el click del stick casi siempre genera algo de deflexion residual, y eso
 *   cancelaba el Homing apenas arrancaba.
 *   Se mantiene deliberadamente FUERA de la maquina de GlobalState (no es un estado nuevo): la
 *   telemetria (T:J1,J2,J3,A1) sigue enviandose sin ningun cambio de formato durante el regreso
 *   a Home, y RECORDING sigue capturandola sin interrupcion -- es literalmente el punto de la
 *   funcionalidad (cerrar la trayectoria grabada en el mismo punto en que arranco).
 *
 * CINTA TRANSPORTADORA (sub-maquina ortogonal, ver Cinta.hpp):
 *   No agrega GlobalStates. La FSM global solo la arma, la desarma y libera el pick:
 *
 *   AutoReady --M120--> cinta armada (Avanzando, o EsperandoPick + EVT:SENSOR si ya hay caja)
 *   Cualquier estado --M121--> cinta deshabilitada (se acepta incluso en Fault/Emergency)
 *   Idle/Manual --M122 [S<v>]--> marcha manual de prueba (S0 detiene)
 *
 *   El frenado ante la deteccion lo decide el firmware (Cinta::update), sin ida y vuelta por
 *   UART. La PC recibe EVT:SENSOR y lanza la trayectoria. Al terminarla (EOF) se marca
 *   _pickCompletado, y la cinta se libera CINTA_RETARDO_LIBERACION_MS despues, ya en AutoReady
 *   (el brazo esta volviendo a Home por el requisito RTH de la PC; en DirectPosition no hay
 *   realimentacion de llegada, por eso el retardo es por tiempo). El rearranque efectivo espera
 *   ademas a que el sensor quede libre (pick fallido => no arranca hasta retirar la caja).
 *
 *   Desarme automatico al entrar a Init, Idle, Manual, Recording, Fault y Emergency, y a
 *   AutoReady salvo que se venga de AutoRunning (fin normal de trayectoria: queda armada
 *   entre cajas) o de PickRecovery/PickAlarm (reintento o continuacion del ciclo). Cada cambio
 *   de estado de la cinta se informa con EVT:CINTA,<ESTADO>.
 *
 * SUPERVISION DEL PICK CON EL SENSOR IR (unico sensor de la estacion):
 *   Activa en AutoRunning cuando la trayectoria arranca con la cinta en EsperandoPick y una caja
 *   frente al IR. Cruza el estado del IR con los flancos de a1 de los puntos consumidos:
 *
 *   Fase          a1   IR esperado   Desvio -> causa
 *   Aproximacion  0    ocupado       IR libre (>= PICK_PERSISTENCIA_MS)       -> CAJA_MOVIDA
 *   Agarre        1    ocupado       IR libre (>= PICK_PERSISTENCIA_MS)       -> CAJA_MOVIDA
 *                                    a1 1->0 con IR ocupado                   -> PLACE_SIN_PICK
 *   Elevacion     1    se libera     IR ocupado tras PICK_T_LIBERACION_MS     -> CAJA_NO_LEVANTADA
 *                                    a1 1->0 con IR ocupado                   -> PLACE_SIN_PICK
 *   Traslado      1    libre         IR ocupado (>= PICK_PERSISTENCIA_MS)     -> CAJA_CAIDA_EN_CINTA
 *   Liberada      0    libre         -
 *   Fin de trayectoria: todavia en Aproximacion                              -> TRAYECTORIA_SIN_SUCCION
 *                       en Agarre o Elevacion                                -> CAJA_NO_LEVANTADA
 *
 *   En Agarre el brazo todavia no se movio: un IR libre ahi no puede ser la caja subiendo (el
 *   brazo no la levanto), sino la caja sacada de la cinta mientras la bomba succionaba.
 *
 *   El pick se confirma (EVT:PICK_OK,<ms desde el inicio de la elevacion>) cuando el IR se libera
 *   en la Elevacion. Ante un desvio: se vacia la FIFO, se purga el efector y el brazo vuelve a Home
 *   con rampa (SmoothPosition desde la ultima consigna). Si al llegar la caja sigue frente al IR y
 *   quedan reintentos (PICK_REINTENTOS_MAX), se re-emite EVT:SENSOR y la PC relanza la misma
 *   trayectoria; si no, PickAlarm espera al operador. El IR decide: sin caja no hay reintento.
 *   TRAYECTORIA_SIN_SUCCION va siempre a PickAlarm: la misma trayectoria fallaria igual.
 *   Limitacion: una caja que cae fuera del haz durante el traslado no es detectable.
 *
 * ESTADO SEGURO POR ESTADO:
 *   Estado              Servos              Efector       Cinta                 Joystick
 *   Init                sostienen posicion  apagado       desarmada             no
 *   Idle                van a Home          sin cambios   desarmada (M122 ok)   no
 *   Manual/Recording    joystick            joystick      desarmada (M122 Man.) si
 *   AutoReady           sostienen posicion  sin cambios   armable (M120)        no
 *   AutoRunning         trayectoria         trayectoria   detenida en el pick   no
 *   AutoPaused          sostienen posicion  sin cambios   detenida              no
 *   PickRecovery        van a Home (rampa)  purga         detenida (armada)     no
 *   PickAlarm           sostienen Home      apagado       detenida (armada)     no
 *   Fault/Emergency     sostienen posicion  apagado       desarmada             no
 *
 *   Fuera de la FSM: el watchdog (IWDG, ~1 s) resetea el micro si el loop se cuelga, y los
 *   handlers de falla (HardFault/Error_Handler) cortan bomba, valvula y cinta (TIM2 CCR=0).
 * ================================================================
 */


// CONSTRUCTOR
Controller::Controller(TerminalSerial& terminal,
                       Joystick& joystickIzquierdo,	Joystick& joystickDerecho,
                       Servo& servoBase, Servo& servoHombro, Servo& servoCodo,
					   Actuador& efectorFinal, Cinta& cinta)
    : _terminal(terminal),
      _joystickIzquierdo(joystickIzquierdo),
	  _joystickDerecho(joystickDerecho),
      _servoBase(servoBase),
	  _servoHombro(servoHombro),
	  _servoCodo(servoCodo),
	  _efectorFinal(efectorFinal),
	  _cinta(cinta),
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
      _chunkRequestTick(0U),
      _endOfTrajectoryPending(false),
      _homingActive(false),
      _pickCompletado(false),
      _tiempoFinTrayectoria(0U),
      _eventoSensorPendiente(false),
      _ultimoEstadoCintaReportado(EstadoCinta::Deshabilitada),
      _fasePick(FasePick::Inactiva),
      _causaFallaPick(CausaFallaPick::Ninguna),
      _intentosPick(0U),
      _tickInicioElevacion(0U),
      _anomaliaPickActiva(false),
      _tickAnomaliaPick(0U),
      _consignaAgarre{0.0f, 0.0f, 0.0f},
      _homeRecuperacionAlcanzado(false),
      _tickHomeRecuperacion(0U),
	  _lastTrajectoryTick(0U),
	  _lastFifoDataTick(0U) {
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

    _pickCompletado = false;
    _eventoSensorPendiente = false;
    _ultimoEstadoCintaReportado = _cinta.getEstado();

    _fasePick = FasePick::Inactiva;
    _causaFallaPick = CausaFallaPick::Ninguna;
    _intentosPick = 0U;

    onEnterState(_state);
}

// UPDATE PRINCIPAL DEL CONTROLADOR
void Controller::update(uint32_t nowMs) {
    if (!_initialized) init();

    // Cinta transportadora: primero, para que el frenado ante la deteccion no espere al
    // parseo de UART. El EVT:SENSOR se difiere hasta despues del ACK del comando en curso.
    if (_cinta.update(nowMs) == EventoCinta::ObjetoDetectado) {
        _eventoSensorPendiente = true;
    }

    // Actualizar la Maquina de estados interna del actuador neumatico
    _efectorFinal.update(nowMs);

    // Transicion de succion (100%) a retencion (10%)
    if (_efectorFinal.getState() == ActuatorState::Suction) {
    	if ((nowMs - _tiempoInicioSuccion) >= 2000U ) {
    		_efectorFinal.commandHold(0.5f);
    		Logger::logDEBUG("Actuador: Vacio asegurado. Pasando a Holding");
    	}
    }

    // Lecutra de Comandos UART
    if (_terminal.hayComandoPendiente()) {
        const char* comando = _terminal.obtenerComandoPendiente();

        if (std::strcmp(comando, "STATUS") == 0) {
            enviarEstadoActualPorUart();
            enviarEstadoCintaPorUart();
        } else {
            const CommandResult result = handleUartLine(comando, nowMs);

            if (result.handled && (result.response[0] != '\0')) {
                _terminal.enviarACK(result.response);
            }
        }

        _terminal.limpiarComandoPendiente();
    }

    // Unico punto de emision de EVT:SENSOR (despues del ACK: la PC recibe ACK_M120 y luego
    // EVT:SENSOR cuando se arma con una caja ya presente).
    if (_eventoSensorPendiente) {
        _terminal.enviarNormal("EVT:SENSOR\r\n");
        _eventoSensorPendiente = false;
    }

    // Liberacion del pick: trayectoria terminada (EOF) y retardo de asentamiento cumplido.
    // No se usa Servo::isTargetReached(): en AutoReady los servos quedan en modo Velocity
    // (detenerActuadoresManuales), donde siempre devuelve false, y en DirectPosition la posicion
    // "actual" salta al setpoint sin reflejar el movimiento fisico del servo.
    if (_pickCompletado && (_state == GlobalState::AutoReady) &&
        ((nowMs - _tiempoFinTrayectoria) >= Config::CINTA_RETARDO_LIBERACION_MS)) {
        _cinta.liberarPick();
        _pickCompletado = false;
        Logger::logDEBUG("CINTA: Trayectoria finalizada. Pick liberado.");
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

    // Informe de cambios de la cinta (una vez por vuelta: captura los cambios de todos los hooks)
    reportarEstadoCinta();
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
        case GlobalState::PickRecovery: return "PickRecovery";
        case GlobalState::PickAlarm:   return "PickAlarm";
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
        case UartCommand::M115_ContinuarCiclo:    return "M115";
        case UartCommand::M116_EndOfTrajectory:   return "M116";
        case UartCommand::M120_ArmarCinta:        return "M120";
        case UartCommand::M121_DesarmarCinta:     return "M121";
        case UartCommand::M122_CintaManual:       return "M122";
        case UartCommand::M999_ResetError:        return "M999";
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
    if (std::strcmp(cmd, "M115") == 0)   return UartCommand::M115_ContinuarCiclo;
    if (std::strcmp(cmd, "M116") == 0)   return UartCommand::M116_EndOfTrajectory;
    if (std::strcmp(cmd, "M120") == 0)   return UartCommand::M120_ArmarCinta;
    if (std::strcmp(cmd, "M121") == 0)   return UartCommand::M121_DesarmarCinta;
    if (std::strncmp(cmd, "M122", 4) == 0) return UartCommand::M122_CintaManual;	// Admite parametro S<v>
    if (std::strcmp(cmd, "M999") == 0)   return UartCommand::M999_ResetError;
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
    _chunkRequestTick = 0U;
    _endOfTrajectoryPending = false;
    _lastTrajectoryTick = 0U;
    _lastFifoDataTick = 0U;
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

    if (_chunkRequestPending) {
        // El watermark superior (24, ver onAutoRunningCommand) libera esta bandera cuando un
        // chunk completo (32 puntos) la cruza sin problema. Pero el ULTIMO chunk de cualquier
        // trayectoria (el resto de dividir el total por el tamaño de chunk) casi nunca es
        // completo, y si es chico puede no cruzar nunca ese watermark -- dejando la bandera
        // trabada para siempre y a la PC sin la oportunidad de responder con M116 (que solo
        // se envia en respuesta a un REQ_CHUNK). Se libera tambien por tiempo como red de
        // seguridad, para no depender de que la aritmetica de watermarks siempre alcance.
        if ((HAL_GetTick() - _chunkRequestTick) < CHUNK_REQUEST_TIMEOUT_MS) {
            return;
        }
        Logger::logDEBUG("AUTO_RUNNING: Solicitud de chunk anterior sin liberar por watermark. Reintentando.");
    }

    Logger::logDEBUG("AUTO_RUNNING: FIFO baja. Enviado REQ_CHUNK.");
    _terminal.enviarNormal("REQ_CHUNK\r\n");
    _chunkRequestPending = true;
    _chunkRequestTick = HAL_GetTick();
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


/*
 * ================================================================
 * SUPERVISION DEL PICK CON EL SENSOR IR
 * ================================================================
 */
namespace {
constexpr float RAD_A_GRADOS = 57.29578f;
}

const char* Controller::toString(CausaFallaPick causa) {
    switch (causa) {
        case CausaFallaPick::CajaMovida:            return "CAJA_MOVIDA";
        case CausaFallaPick::CajaNoLevantada:       return "CAJA_NO_LEVANTADA";
        case CausaFallaPick::PlaceSinPick:          return "PLACE_SIN_PICK";
        case CausaFallaPick::CajaCaidaEnCinta:      return "CAJA_CAIDA_EN_CINTA";
        case CausaFallaPick::TrayectoriaSinSuccion: return "TRAYECTORIA_SIN_SUCCION";
        default:                                    return "NINGUNA";
    }
}

void Controller::iniciarSupervisionPick(void) {
    // Solo dentro del ciclo: trayectoria disparada por una caja frente al IR con la cinta
    // detenida esperando el pick. Fuera de eso (p. ej. ensayos sin caja) no hay nada que verificar.
    const bool enCiclo = (_cinta.getEstado() == EstadoCinta::EsperandoPick) && _cinta.objetoPresente();
    _fasePick = enCiclo ? FasePick::Aproximacion : FasePick::Inactiva;
    _anomaliaPickActiva = false;
}

void Controller::supervisarPickPunto(const PendingAutoCommand& punto, uint32_t nowMs) {
    const bool efectorEncendido = (punto.a1 != 0U);

    switch (_fasePick) {
        case FasePick::Aproximacion:
            if (efectorEncendido) {                         // Flanco 0->1: agarre
                _consignaAgarre[0] = punto.q1 * RAD_A_GRADOS;
                _consignaAgarre[1] = punto.q2 * RAD_A_GRADOS;
                _consignaAgarre[2] = punto.q3 * RAD_A_GRADOS;
                _fasePick = FasePick::Agarre;
            }
            break;

        case FasePick::Agarre:
        case FasePick::Elevacion:
            if (!efectorEncendido) {                        // Flanco 1->0 sin pick confirmado
                if (_cinta.objetoPresente()) {
                    fallarPick(CausaFallaPick::PlaceSinPick);       // La caja nunca salio de la cinta
                } else if (_fasePick == FasePick::Agarre) {
                    fallarPick(CausaFallaPick::CajaMovida);         // Brazo quieto y sin caja: la sacaron
                } else {
                    confirmarPick(nowMs);                           // El IR se libero en este mismo ciclo
                    _fasePick = FasePick::Liberada;
                }
            } else if (_fasePick == FasePick::Agarre) {
                // Primer movimiento posterior al agarre: arranca el plazo de liberacion del IR
                const float umbral = Config::PICK_UMBRAL_MOVIMIENTO_GRADOS;
                if ((std::fabs(punto.q1 * RAD_A_GRADOS - _consignaAgarre[0]) > umbral) ||
                    (std::fabs(punto.q2 * RAD_A_GRADOS - _consignaAgarre[1]) > umbral) ||
                    (std::fabs(punto.q3 * RAD_A_GRADOS - _consignaAgarre[2]) > umbral)) {
                    _fasePick = FasePick::Elevacion;
                    _tickInicioElevacion = nowMs;
                }
            }
            break;

        case FasePick::Traslado:
            if (!efectorEncendido) {
                _fasePick = FasePick::Liberada;             // Place: la caja ya salio de la cinta
            }
            break;

        default:
            break;
    }
}

void Controller::supervisarPickSensor(uint32_t nowMs) {
    const bool ocupado = _cinta.objetoPresente();

    switch (_fasePick) {
        case FasePick::Elevacion:
            if (!ocupado) {
                confirmarPick(nowMs);                       // El IR se libero con vacio: pick OK
            } else if ((nowMs - _tickInicioElevacion) >= Config::PICK_T_LIBERACION_MS) {
                fallarPick(CausaFallaPick::CajaNoLevantada);
            }
            return;

        case FasePick::Aproximacion:
        case FasePick::Agarre:
        case FasePick::Traslado: {
            // Desvios que deben sostenerse PICK_PERSISTENCIA_MS (filtra parpadeos del FC-51).
            // Antes de levantar la caja (bajada o succion con el brazo quieto) el IR debe verla;
            // despues de confirmar el pick, no.
            const bool desvio = (_fasePick == FasePick::Traslado) ? ocupado : !ocupado;
            if (!desvio) {
                _anomaliaPickActiva = false;
            } else if (!_anomaliaPickActiva) {
                _anomaliaPickActiva = true;
                _tickAnomaliaPick = nowMs;
            } else if ((nowMs - _tickAnomaliaPick) >= Config::PICK_PERSISTENCIA_MS) {
                fallarPick((_fasePick == FasePick::Traslado) ? CausaFallaPick::CajaCaidaEnCinta
                                                             : CausaFallaPick::CajaMovida);
            }
            return;
        }

        default:
            return;
    }
}

void Controller::confirmarPick(uint32_t nowMs) {
    // Tiempo desde el primer movimiento hasta que el IR dejo de ver la caja (calibra T_lib)
    const uint32_t ms = (_fasePick == FasePick::Elevacion) ? (nowMs - _tickInicioElevacion) : 0U;
    _fasePick = FasePick::Traslado;
    _anomaliaPickActiva = false;
    _intentosPick = 0U;

    char msg[32];
    std::snprintf(msg, sizeof(msg), "EVT:PICK_OK,%lu\r\n", static_cast<unsigned long>(ms));
    _terminal.enviarNormal(msg);
}

void Controller::fallarPick(CausaFallaPick causa) {
    _causaFallaPick = causa;
    if (_intentosPick < 255U) _intentosPick++;
    transitionTo(GlobalState::PickRecovery, HAL_GetTick());
}

void Controller::continuarCicloTrasPick(void) {
    // Decision del operador: arranca un conteo nuevo de reintentos para la caja que siga
    _intentosPick = 0U;
    transitionTo(GlobalState::AutoReady, HAL_GetTick());

    if (_cinta.getEstado() != EstadoCinta::EsperandoPick) return;   // Cinta desarmada (M121)

    if (_cinta.objetoPresente()) {
        _eventoSensorPendiente = true;  // Caja reacomodada: la PC relanza el pick
    } else {
        _cinta.liberarPick();           // Caja retirada a mano: la cinta avanza a la siguiente
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
        case GlobalState::PickRecovery: onEnterPickRecovery(); break;
        case GlobalState::PickAlarm:   onEnterPickAlarm(); break;
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
        case GlobalState::PickRecovery: onUpdatePickRecovery(); break;
        case GlobalState::PickAlarm:   onUpdatePickAlarm(); break;
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

    if (command == UartCommand::M999_ResetError) {
        // Salida controlada de Fault/Emergency. Se evalua con la misma prioridad que M00,
        // ANTES del bloqueo incondicional de esos dos estados (si no, ese mismo bloqueo se
        // comeria a M999). Solo valido viniendo de Fault/Emergency, y solo tras una ventana
        // minima de permanencia (evita que un reflejo -- apretar el mismo boton fisico que
        // ahora dice RESET -- limpie una falla real sin que nadie evaluo la escena).
        const bool enEstadoDeFalla = (_state == GlobalState::Fault) || (_state == GlobalState::Emergency);
        if (!enEstadoDeFalla) {
            return makeErr("ERR:INVALID_STATE");
        }
        if ((nowMs - _stateEntryTick) < M999_MIN_DWELL_MS) {
            return makeErr("ERR:RESET_TOO_SOON");
        }
        transitionTo(GlobalState::Init, nowMs);
        return makeAck("ACK_M999");
    }

    if (command == UartCommand::M121_DesarmarCinta) {
        // Detener la cinta es siempre seguro: se acepta en cualquier estado, incluso en
        // Fault/Emergency (por eso se evalua antes del bloqueo de esos dos estados).
        _cinta.desarmar();
        _pickCompletado = false;
        return makeAck("ACK_M121");
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

    if (command == UartCommand::M122_CintaManual) {
        // Marcha manual de prueba: solo fuera del ciclo automatico y sin teleoperacion grabando
        if ((_state == GlobalState::Idle) || (_state == GlobalState::Manual)) {
            return procesarComandoCintaManual(rawLine, nowMs);
        }
        return makeErr("ERR:INVALID_STATE");
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
                // La trayectoria grabada debe empezar y terminar en Home (requisito RTH de la PC)
                if (_homingActive || !robotEnHome()) {
                    return makeErr("ERR:NOT_AT_HOME");
                }
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
            if (command == UartCommand::M120_ArmarCinta) {
                _intentosPick = 0U;     // Ciclo nuevo: la primera caja arranca con reintentos completos
                // Si ya hay una caja frente al sensor, su flanco ocurrio antes de armar: se
                // re-emite EVT:SENSOR (despues del ACK) para que la PC no quede esperando.
                if (_cinta.armar(nowMs) == EventoCinta::ObjetoDetectado) {
                    _eventoSensorPendiente = true;
                } else if ((_cinta.getEstado() == EstadoCinta::EsperandoPick) && _cinta.objetoPresente()) {
                    // Re-armado idempotente con la caja esperando: re-emitir para que la PC
                    // pueda recuperarse si perdio el EVT:SENSOR original.
                    _eventoSensorPendiente = true;
                }
                return makeAck("ACK_M120");
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
            if (command == UartCommand::M116_EndOfTrajectory) {
                // No hay mas puntos en camino: no cambiamos de estado todavia, solo dejamos de
                // pedir chunks y drenamos la FIFO restante. La transicion a AutoReady es automatica
                // (ver onUpdateAutoRunning) una vez que la FIFO se vacie.
                _endOfTrajectoryPending = true;
                Logger::logDEBUG("AUTO_RUNNING: EOF marcado. Drenando FIFO restante...");
                return makeAck("ACK_M116");
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

        case GlobalState::PickRecovery:
            // Recuperacion automatica en curso (homing con rampa). Solo se acepta salir a Idle.
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            if (command == UartCommand::M116_EndOfTrajectory) {
                // EOF que la PC envio antes de enterarse del aborto: ya no hay nada que drenar.
                return makeAck("ACK_M116");
            }
            return makeErr("ERR:PICK_RECOVERY");

        case GlobalState::PickAlarm:
            if (command == UartCommand::M115_ContinuarCiclo) {
                continuarCicloTrasPick();
                return makeAck("ACK_M115");
            }
            if (command == UartCommand::M113_StopPausePlayback) {
                // Cancelar: el ciclo termina (misma semantica que CANCELAR desde AutoPaused)
                transitionTo(GlobalState::AutoReady, nowMs);
                _cinta.desarmar();
                _pickCompletado = false;
                _intentosPick = 0U;
                return makeAck("ACK_M113");
            }
            if (command == UartCommand::M99_GoIdle) {
                transitionTo(GlobalState::Idle, nowMs);
                return makeAck("ACK_M99");
            }
            if (command == UartCommand::M116_EndOfTrajectory) {
                return makeAck("ACK_M116");
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
    _homingActive = false;
}

void Controller::realizarServoInitInicial(void) {
    /* Intencionalmente vacío */
}

void Controller::realizarGoHomeNormal(void) {
    _servoBase.goHome();
    _servoHombro.goHome();
    _servoCodo.goHome();
}

bool Controller::robotEnHome(void) const {
    return (std::fabs(_servoBase.getAngleDegrees()   - Config::SERVO_BASE_HOME_ANGLE)   <= Config::HOME_TOLERANCIA_GRADOS) &&
           (std::fabs(_servoHombro.getAngleDegrees() - Config::SERVO_HOMBRO_HOME_ANGLE) <= Config::HOME_TOLERANCIA_GRADOS) &&
           (std::fabs(_servoCodo.getAngleDegrees()   - Config::SERVO_CODO_HOME_ANGLE)   <= Config::HOME_TOLERANCIA_GRADOS);
}

void Controller::enviarTelemetria(void) {
	// Buffer de mensaje
    char msg[96];

    // Estado de Servos
    const float j1_deg = _servoBase.getAngleDegrees();
    const float j2_deg = _servoHombro.getAngleDegrees();
    const float j3_deg = _servoCodo.getAngleDegrees();

    // Estado de Actuador
    int a1 = 0;
    ActuatorState estadoActuador = _efectorFinal.getState();
    if (estadoActuador == ActuatorState::Suction || estadoActuador == ActuatorState::Holding) {
    	a1 = 1;
    }

    std::snprintf(msg, sizeof(msg), "T:%.2f,%.2f,%.2f,%d\r\n", j1_deg, j2_deg, j3_deg, a1);
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

void Controller::enviarEstadoCintaPorUart(void) {
    char msg[40];
    std::snprintf(msg, sizeof(msg), "EVT:CINTA,%s\r\n", Cinta::toString(_cinta.getEstado()));
    _terminal.enviarNormal(msg);
}

void Controller::reportarEstadoCinta(void) {
    const EstadoCinta estado = _cinta.getEstado();
    if (estado == _ultimoEstadoCintaReportado) return;

    _ultimoEstadoCintaReportado = estado;
    enviarEstadoCintaPorUart();
}

CommandResult Controller::procesarComandoCintaManual(const char* rawCommand, uint32_t nowMs) {
    char cmd[32] = {0};
    normalizeToUpperNoSpaces(rawCommand, cmd, sizeof(cmd));

    // "M122" (velocidad nominal) o "M122S<v>" con v en [0.0, 1.0]
    float velocidad = Config::CINTA_VELOCIDAD_NOMINAL;

    if (cmd[4] == 'S') {
        char* endPtr = nullptr;
        velocidad = std::strtof(&cmd[5], &endPtr);
        // !(rango) en lugar de (<0 || >1): tambien rechaza NaN ("M122SNAN")
        if ((endPtr == &cmd[5]) || (*endPtr != '\0') || !((velocidad >= 0.0f) && (velocidad <= 1.0f))) {
            return makeErr("ERR:PARAM");
        }
    } else if (cmd[4] != '\0') {
        return makeErr("ERR:PARAM");
    }

    if (!_cinta.setVelocidadManual(velocidad, nowMs)) {
        return makeErr("ERR:SENSOR_ACTIVO");
    }

    return makeAck("ACK_M122");
}


void Controller::procesarTeleoperacion(void) {
    // Ambos pulsadores se evaluan siempre (una vez por tick cada uno) para consumir
    // correctamente su FSM de antirrebote, independientemente de si en este momento se
    // actua sobre el resultado o no (ver bloqueo total durante Homing, mas abajo).
    const bool pulsadorIzquierdoValido = _joystickIzquierdo.HayPulsacionValida();
    const bool pulsadorDerechoValido = _joystickDerecho.HayPulsacionValida();

    // Disparo de Homing: pulsador izquierdo (libre en teleoperacion) inicia el regreso a Home.
    // Necesario para que una trayectoria grabada manualmente pueda cerrar en el mismo punto en
    // que arranco (Return-To-Home), algo impracticable a mano con precision via joystick.
    if (!_homingActive && pulsadorIzquierdoValido) {
        Logger::logDEBUG("TELEOPERACION: Homing solicitado por pulsador izquierdo.");
        _servoBase.goHome();
        _servoHombro.goHome();
        _servoCodo.goHome();
        _homingActive = true;
    }

    if (_homingActive) {
        // Bloqueo total de la teleoperacion (ejes Y pulsador de actuador) mientras dura el
        // Homing, sin cancelacion por movimiento: presionar el click del joystick casi siempre
        // genera algo de deflexion residual del propio stick, y con el loop principal corriendo
        // sin limite de velocidad, una cancelacion "por movimiento" abortaba el Homing apenas
        // arrancaba (en la misma fraccion de milisegundo, antes de mover nada). Se sale de este
        // sub-estado unicamente cuando los 3 servos reportan isTargetReached().
        if (_servoBase.isTargetReached() && _servoHombro.isTargetReached() && _servoCodo.isTargetReached()) {
            Logger::logDEBUG("TELEOPERACION: Homing completado.");
            _homingActive = false;
        }
        return;
    }

    // 1. Actualizar Motores pasando comando de velocidad
    _servoBase.setTargetVelocity(_joystickIzquierdo.ObtenerX());
    _servoHombro.setTargetVelocity(_joystickIzquierdo.ObtenerY());
    _servoCodo.setTargetVelocity(_joystickDerecho.ObtenerY());

    // 2. Actualizar Actuador si hay pulsación
    if (pulsadorDerechoValido) {
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

    // Estado seguro de espera: sin energia en el efector (simetrico con Fault/Emergency)
    _efectorFinal.commandIdle();

    clearAutoRuntimeState();
    _cinta.desarmar();
    _pickCompletado = false;
    _fasePick = FasePick::Inactiva;
    _intentosPick = 0U;
}

void Controller::onEnterIdle(void) {
    Logger::logDEBUG("FSM -> Idle");
    detenerActuadoresManuales();
    clearAutoRuntimeState();
    _cinta.desarmar();
    _pickCompletado = false;
    _fasePick = FasePick::Inactiva;
    _intentosPick = 0U;

    if (_primerIngresoIdle) {
        realizarServoInitInicial();
        _primerIngresoIdle = false;
    } else {
        realizarGoHomeNormal();
    }
}

void Controller::onEnterManual(void) {
    Logger::logDEBUG("FSM -> Manual");
    _cinta.desarmar();
    _pickCompletado = false;
}

void Controller::onEnterRecording(void) {
    Logger::logDEBUG("FSM -> Recording");
    // En Recording el operador teleopera y graba: la cinta no debe moverse bajo el brazo
    _cinta.desarmar();
    _pickCompletado = false;
    // Sin homing: la guarda de M110 garantiza que la grabacion arranca en Home
}

void Controller::onEnterAutoReady(void) {
    Logger::logDEBUG("FSM -> AutoReady");
    detenerActuadoresManuales();
    clearAutoRuntimeState();

    _fasePick = FasePick::Inactiva;

    // Fin normal de trayectoria (AutoRunning -> AutoReady): la cinta sigue armada entre cajas y
    // se libera via _pickCompletado. Lo mismo al salir de PickRecovery (reintento) o de
    // PickAlarm (M115 continua; M113 la desarma explicitamente). Desde cualquier otro origen
    // (M101 desde Idle, cancelacion M113 desde AutoPaused) el ciclo se considera terminado.
    const bool cicloSigue = (_previousState == GlobalState::AutoRunning) ||
                            (_previousState == GlobalState::PickRecovery) ||
                            (_previousState == GlobalState::PickAlarm);
    if (!cicloSigue) {
        _cinta.desarmar();
        _pickCompletado = false;
    }
}

void Controller::onEnterAutoRunning(void) {
    Logger::logDEBUG("FSM -> AutoRunning");

    /*
     * Al iniciar playback desde AutoReady queremos arrancar limpio.
     * Si venimos desde AutoPaused por M114, preservamos FIFO y punto activo.
     */
    if (_previousState != GlobalState::AutoPaused) {
        clearAutoRuntimeState();
        // Una trayectoria nueva invalida cualquier liberacion pendiente (p. ej. reintento de un
        // pick fallido: la cinta no debe arrancar cuando el brazo levante la caja).
        _pickCompletado = false;
        _cinta.cancelarLiberacion();
        _lastTrajectoryTick = HAL_GetTick(); // Inicia el reloj principal
        _lastFifoDataTick = HAL_GetTick();   // Arranca limpio el reloj de subalimentacion
        iniciarSupervisionPick();
        Logger::logDEBUG("AUTO_RUNNING: FIFO inicializada. Listo para solicitar 1er chunk...");
    } else {
    	_lastTrajectoryTick = HAL_GetTick(); // Resincroniza el reloj al despausar
    	_lastFifoDataTick = HAL_GetTick();   // Idem: evita un falso timeout apenas se reanuda
        // La pausa no consume el plazo de elevacion ni la persistencia de un desvio del IR
        _tickInicioElevacion = HAL_GetTick();
        _anomaliaPickActiva = false;
        Logger::logDEBUG("AUTO_RUNNING: reanudando ejecucion. FIFO preservada.");
    }
}

void Controller::onEnterAutoPaused(void) {
    Logger::logDEBUG("FSM -> AutoPaused");
    detenerActuadoresManuales();
}

void Controller::onEnterPickRecovery(void) {
    Logger::logERROR("FSM -> PickRecovery (%s, intento %u)", toString(_causaFallaPick),
                     static_cast<unsigned>(_intentosPick));

    // Abortar la trayectoria: no se consumen mas puntos y los G1 en vuelo se ignoran
    clearAutoRuntimeState();
    _fasePick = FasePick::Inactiva;

    // Purgar el vacio (si la ventosa tomo la caja a medias, la suelta sobre la cinta)
    const ActuatorState efector = _efectorFinal.getState();
    if ((efector == ActuatorState::Suction) || (efector == ActuatorState::Holding)) {
        _efectorFinal.commandRelease();
    }

    // Regreso a Home con rampa trapezoidal desde la ultima consigna de DirectPosition
    realizarGoHomeNormal();
    _homeRecuperacionAlcanzado = false;

    // La cinta queda detenida y armada (EsperandoPick): sin liberacion pendiente
    _cinta.cancelarLiberacion();
    _pickCompletado = false;

    char msg[48];
    std::snprintf(msg, sizeof(msg), "EVT:PICK_FAIL,%s,%u\r\n", toString(_causaFallaPick),
                  static_cast<unsigned>(_intentosPick));
    _terminal.enviarNormal(msg);
}

void Controller::onEnterPickAlarm(void) {
    Logger::logERROR("FSM -> PickAlarm (%s). Esperando al operador (M115 / M113).",
                     toString(_causaFallaPick));
    char msg[48];
    std::snprintf(msg, sizeof(msg), "EVT:PICK_ALARM,%s\r\n", toString(_causaFallaPick));
    _terminal.enviarNormal(msg);
}

void Controller::onEnterFault(void) {
    Logger::logERROR("FSM -> Fault");
    detenerActuadoresManuales();

    // Cortar energia a actuador final (simetrico con onEnterEmergency: cualquier estado de
    // seguridad debe soltar/despresurizar por defecto, no mantener lo que tenia).
    _efectorFinal.commandIdle();

    // Detener la cinta: ningun estado de seguridad debe dejar material en movimiento.
    _cinta.desarmar();
    _pickCompletado = false;

    clearAutoRuntimeState();
    _terminal.enviarNormal("EVT:FAULT\r\n");
}

void Controller::onEnterEmergency(void) {
    Logger::logERROR("FSM -> Emergency");
    detenerActuadoresManuales();

    // Cortar energia a actuador final
    _efectorFinal.commandIdle();

    // Detener la cinta de inmediato (frenado sin rampa)
    _cinta.desarmar();
    _pickCompletado = false;

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

    // Supervision continua del IR (caja retirada, plazo de elevacion, caja caida)
    supervisarPickSensor(nowMs);
    if (_state != GlobalState::AutoRunning) return;

    // Metrónomo estricto a 50Hz (20ms)
    if ((nowMs - _lastTrajectoryTick) >= 20U) {

        // Sumar exactamente 20 elimina el drift acumulativo
        _lastTrajectoryTick += 20U;

        PendingAutoCommand currentPoint{};

        if (dequeueAutoPoint(currentPoint)) {
            // Datos reales fluyendo: refrescamos el reloj de subalimentacion.
            _lastFifoDataTick = nowMs;

            // Supervision del punto ANTES de aplicarlo: si aborta, el punto no se ejecuta
            supervisarPickPunto(currentPoint, nowMs);
            if (_state != GlobalState::AutoRunning) return;

            // A. Cinemática
            float anguloGradosQ1 = currentPoint.q1 * 57.29578f;
            float anguloGradosQ2 = currentPoint.q2 * 57.29578f;
            float anguloGradosQ3 = currentPoint.q3 * 57.29578f;

            _servoBase.setDirectTargetPosition(anguloGradosQ1);
            _servoHombro.setDirectTargetPosition(anguloGradosQ2);
            _servoCodo.setDirectTargetPosition(anguloGradosQ3);

            // B. Actuador (Corregido con timestamp)
            gestionarActuadorAuto(currentPoint.a1, nowMs);
        } else if (_endOfTrajectoryPending) {
            // Fin de trayectoria sin pick confirmado: la caja sigue frente al IR
            if (_fasePick == FasePick::Aproximacion) {
                fallarPick(CausaFallaPick::TrayectoriaSinSuccion);  // Nunca encendio el efector
                return;
            }
            if ((_fasePick == FasePick::Agarre) || (_fasePick == FasePick::Elevacion)) {
                fallarPick(CausaFallaPick::CajaNoLevantada);
                return;
            }

            // FIFO vacia y confirmado que no llegaran mas puntos (M116): trayectoria completa.
            Logger::logDEBUG("AUTO_RUNNING: FIFO vacia con EOF confirmado. Trayectoria completa.");
            _pickCompletado = true;     // La cinta se libera tras CINTA_RETARDO_LIBERACION_MS
            _tiempoFinTrayectoria = nowMs;
            transitionTo(GlobalState::AutoReady, nowMs);
            _terminal.enviarNormal("EVT:AUTO_COMPLETE\r\n");
            return;
        }
        // Underflow sin EOF pendiente: pueden llegar mas puntos. Si la PC dejo de responder a
        // REQ_CHUNK por demasiado tiempo (perdio la conexion, se colgo, etc.), esto ya no es
        // una espera normal -- pasamos a Fault en vez de esperar en silencio indefinidamente.
        else if ((nowMs - _lastFifoDataTick) >= AUTO_FIFO_STARVATION_TIMEOUT_MS) {
            Logger::logERROR("AUTO_RUNNING: Timeout de FIFO. Posible perdida de comunicacion con la PC.");
            forceFault(nowMs);
            return;
        }
    }

    // Evaluación en background. Sin sentido pedir mas chunks si ya sabemos que no llegaran.
    if (!_endOfTrajectoryPending) {
        requestNextChunkIfNeeded();
    }
}

void Controller::onUpdateAutoPaused(void) {}

void Controller::onUpdatePickRecovery(void) {
    const uint32_t nowMs = HAL_GetTick();

    // 1. Esperar el regreso a Home (SmoothPosition informa la llegada) y el asentamiento
    if (!_homeRecuperacionAlcanzado) {
        if (_servoBase.isTargetReached() && _servoHombro.isTargetReached() && _servoCodo.isTargetReached()) {
            _homeRecuperacionAlcanzado = true;
            _tickHomeRecuperacion = nowMs;
        }
        return;
    }
    if ((nowMs - _tickHomeRecuperacion) < Config::PICK_ASENTAMIENTO_MS) return;

    // 2. Decide el IR: con la caja todavia en posicion y reintentos disponibles, se relanza el
    //    mismo pick (la PC recibe EVT:SENSOR y vuelve a enviar la trayectoria del paso actual).
    //    Una trayectoria sin succion no se reintenta: la misma trayectoria fallaria igual.
    const bool cajaPresente = _cinta.objetoPresente();
    const bool cicloArmado = (_cinta.getEstado() == EstadoCinta::EsperandoPick);
    const bool reintentable = (_causaFallaPick != CausaFallaPick::TrayectoriaSinSuccion);

    if (reintentable && cajaPresente && cicloArmado && (_intentosPick <= Config::PICK_REINTENTOS_MAX)) {
        char msg[32];
        std::snprintf(msg, sizeof(msg), "EVT:PICK_RETRY,%u\r\n", static_cast<unsigned>(_intentosPick));
        _terminal.enviarNormal(msg);
        transitionTo(GlobalState::AutoReady, nowMs);
        _eventoSensorPendiente = true;      // Se emite en la proxima vuelta, despues de PICK_RETRY
    } else {
        transitionTo(GlobalState::PickAlarm, nowMs);
    }
}

void Controller::onUpdatePickAlarm(void) {}

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
