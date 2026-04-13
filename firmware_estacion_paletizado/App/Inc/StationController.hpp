#ifndef STATION_CONTROLLER_HPP
#define STATION_CONTROLLER_HPP

#include "Joystick.hpp"
#include "Servo.hpp"
#include "TerminalSerial.hpp"
#include <cstdint>

/*
 * ================================================================
 * STATION CONTROLLER
 * ================================================================
 *
 * Este módulo concentra la lógica principal de la estación:
 * - interpretación de comandos UART
 * - máquina de estados finitos (FSM)
 * - acciones al entrar a cada estado
 * - acciones periódicas por estado
 * - telemetría
 * - respuestas ACK / ERR del protocolo
 *
 * NUEVO CRITERIO DE ARRANQUE:
 * - Boot -> Init ocurre automáticamente
 * - Init ya NO pasa automáticamente a Idle
 * - Init queda como estado seguro de espera
 * - M99 permite pasar de Init a Idle
 * - M98 permite volver a Init desde:
 *      * Idle
 *      * Manual
 *      * AutoReady
 * ================================================================
 */

namespace Station {

/*
 * ------------------------------------------------
 * Estados globales de la estación
 * ------------------------------------------------
 */
enum class GlobalState {
    Boot,
    Init,
    Idle,
    Manual,
    Recording,
    AutoReady,
    AutoRunning,
    AutoPaused,
    Fault,
    Emergency
};

/*
 * ------------------------------------------------
 * Comandos reconocidos por UART
 * ------------------------------------------------
 */
enum class UartCommand {
    None,
    Unknown,

    M98_GoInit,
    M99_GoIdle,
    M100_ManualMode,
    M101_AutoMode,
    M110_StartRecording,
    M111_StopRecording,
    M112_StartPlayback,
    M113_StopPausePlayback,
    M114_ResumePlayback,
    M00_EStop,

    Status
};

/*
 * ------------------------------------------------
 * Resultado del procesamiento de un comando
 * ------------------------------------------------
 *
 * handled  -> indica si la FSM procesó el comando
 * success  -> indica si el resultado fue exitoso
 * response -> mensaje de respuesta a enviar
 */
struct CommandResult {
    bool handled;
    bool success;
    char response[48];
};

/*
 * ------------------------------------------------
 * Comando / punto automático parseado desde G:q1,q2,q3,a1,a2
 * ------------------------------------------------
 */
struct PendingAutoCommand {
    bool valid;
    float q1;
    float q2;
    float q3;
};

/*
 * ------------------------------------------------
 * Clase principal del controlador de estación
 * ------------------------------------------------
 */
class Controller {
public:
    Controller(TerminalSerial& terminal,
               Joystick& joystickIzquierdo,
               Servo& servoBase);

    void init(void);
    void update(uint32_t nowMs);

    GlobalState getState(void) const;
    GlobalState getPreviousState(void) const;
    const char* getStateName(void) const;

    static const char* stateToString(GlobalState state);

    bool didStateChange(void) const;
    void clearStateChangeFlag(void);

    void forceFault(uint32_t nowMs);
    void forceEmergency(uint32_t nowMs);

private:
    /*
     * ------------------------------------------------
     * Referencias a recursos externos
     * ------------------------------------------------
     */
    TerminalSerial& _terminal;
    Joystick& _joystickIzquierdo;
    Servo& _servoBase;

    /*
     * ------------------------------------------------
     * Estado interno
     * ------------------------------------------------
     */
    GlobalState _state;
    GlobalState _previousState;

    bool _stateChanged;
    bool _initialized;
    bool _primerIngresoIdle;

    uint32_t _stateEntryTick;
    uint32_t _tiempoUltimoEnvioTelemetria;

    /*
     * ------------------------------------------------
     * NUEVO: FIFO de puntos para AUTO_RUNNING
     * ------------------------------------------------
     */
    static constexpr uint8_t AUTO_FIFO_CAPACITY = 16U;
    PendingAutoCommand _autoFifo[AUTO_FIFO_CAPACITY];
    uint8_t _autoFifoHead;
    uint8_t _autoFifoTail;
    uint8_t _autoFifoCount;

    PendingAutoCommand _activeAutoPoint;
    bool _hasActiveAutoPoint;
    bool _chunkRequestPending;

    // NUEVO:
    // true cuando se acaba de entrar a AUTO_RUNNING desde AUTO_READY
    // y todavia no llego el primer chunk enviado por Python tras ACK_M112.
    bool _waitingFirstChunkAfterStart;

private:
    /*
     * ================================================================
     * UTILIDADES GENERALES
     * ================================================================
     */
    static const char* toString(GlobalState state);
    static const char* toString(UartCommand command);

    static void normalizeToUpperNoSpaces(const char* src, char* dst, uint16_t dstSize);
    static UartCommand parseUartCommand(const char* text);

    CommandResult makeAck(const char* ackText);
    CommandResult makeErr(const char* errText);
    CommandResult makeHandledNoReply(void);
    CommandResult makeNotHandled(void);

    /*
     * ================================================================
     * Parser local para G1:q1,q2,q3
     * ================================================================
     */
    bool tryParseGCommand(const char* rawLine, PendingAutoCommand& outCmd);

    /*
     * ================================================================
     * Helpers FIFO para AUTO_RUNNING
     * ================================================================
     */
    bool enqueueAutoPoint(const PendingAutoCommand& cmd);
    bool dequeueAutoPoint(PendingAutoCommand& cmd);
    bool isAutoFifoEmpty(void) const;
    bool isAutoFifoFull(void) const;
    void clearAutoFifo(void);
    void clearAutoRuntimeState(void);
    void startNextAutoPointIfNeeded(void);
    void processActiveAutoPoint(void);
    void requestNextChunkIfNeeded(void);

    /*
     * ================================================================
     * FSM
     * ================================================================
     */
    void transitionTo(GlobalState newState, uint32_t nowMs);
    void onEnterState(GlobalState state);
    void runCurrentState(void);
    CommandResult handleUartLine(const char* rawLine, uint32_t nowMs);
    void processStateLocalCommand(const char* rawLine);

    /*
     * ================================================================
     * AYUDAS DE LA ESTACIÓN
     * ================================================================
     */
    void detenerActuadoresManuales(void);
    void realizarServoInitInicial(void);
    void realizarGoHomeNormal(void);
    void enviarTelemetria(void);
    void enviarEstadoActualPorUart(void);

    /*
     * ================================================================
     * CALLBACKS DE ENTRADA A ESTADO
     * ================================================================
     */
    void onEnterBoot(void);
    void onEnterInit(void);
    void onEnterIdle(void);
    void onEnterManual(void);
    void onEnterRecording(void);
    void onEnterAutoReady(void);
    void onEnterAutoRunning(void);
    void onEnterAutoPaused(void);
    void onEnterFault(void);
    void onEnterEmergency(void);

    /*
     * ================================================================
     * CALLBACKS DE UPDATE POR ESTADO
     * ================================================================
     */
    void onUpdateInit(void);
    void onUpdateIdle(void);
    void onUpdateManual(void);
    void onUpdateRecording(void);
    void onUpdateAutoReady(void);
    void onUpdateAutoRunning(void);
    void onUpdateAutoPaused(void);
    void onUpdateFault(void);
    void onUpdateEmergency(void);

    /*
     * ================================================================
     * CALLBACKS DE COMANDOS LOCALES
     * ================================================================
     */
    void onManualCommand(const char* rawCommand);
    void onRecordingCommand(const char* rawCommand);
    void onAutoReadyCommand(const char* rawCommand);
    void onAutoRunningCommand(const char* rawCommand);
    void onAutoPausedCommand(const char* rawCommand);
    void onIdleCommand(const char* rawCommand);
};

} // namespace Station

#endif
