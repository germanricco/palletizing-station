/**
 * @file StationController.hpp
 * @brief Controlador central y Máquina de Estados (FSM) de la Estación de Paletizado.
 * @details Este módulo concentra la lógica principal de la estación:
 *          - Interpretación de comandos UART.
 *          - Máquina de Estados Finitos (FSM).
 *          - Ejecución de trayectorias (Streaming) y telemetría.
 *          - Respuestas ACK/ERR del protocolo.
 */

#ifndef STATION_CONTROLLER_HPP
#define STATION_CONTROLLER_HPP

#include "Joystick.hpp"
#include "Servo.hpp"
#include "Actuador.hpp"
#include "TerminalSerial.hpp"
#include <cstdint>



namespace Station {

/**
 * @enum GlobalState
 * @brief Estados operativos de la Máquina de Estados Finita (FSM).
 */
enum class GlobalState {
    Boot,			/**< Arranque del sistema y configuración de periféricos. */
    Init,			/**< Estado seguro de espera inicial. Actuadores bloqueados. */
    Idle,			/**< Estación en reposo, lista para recibir comandos operativos. */
    Manual,			/**< Control directo teleoperado por el usuario mediante Joystick. */
    Recording,		/**< Modo de enseñanza (Teach-in). Guarda puntos de trayectoria. */
    AutoReady,		/**< Preparado para recibir y ejecutar trayectoria G-Code. */
    AutoRunning,	/**< Ejecución activa de trayectoria en tiempo real (Streaming). */
    AutoPaused,		/**< Pausa de trayectoria. Mantiene posición sin vaciar la FIFO. */
    Fault,			/**< Falla recuperable del sistema. */
    Emergency		/**< Parada de emergencia (E-Stop). Requiere reinicio físico. */
};


/**
 * @enum UartCommand
 * @brief Comandos de control estructurados reconocidos por el protocolo UART.
 */
enum class UartCommand {
    None,
    Unknown,

    M98_GoInit,				/**< Retorno forzado al estado seguro (Init). */
    M99_GoIdle,				/**< Transición al estado de reposo operativo (Idle). */
    M100_ManualMode,		/**< Habilita el control por joystick. */
    M101_AutoMode,			/**< Prepara el buffer para ejecución automática. */
    M110_StartRecording,	/**< Inicia el streaming de telemetría para guardado. */
    M111_StopRecording,		/**< Finaliza el guardado y retorna a Manual. */
    M112_StartPlayback,		/**< Inicia el consumo de la FIFO de trayectoria. */
    M113_StopPausePlayback,	/**< Pausa la ejecución (desde AutoRunning) o cancela (desde AutoPaused). */
    M114_ResumePlayback,	/**< Reanuda la trayectoria pausada. */
    M116_EndOfTrajectory,	/**< Marca que no llegarán más puntos: drenar FIFO y pasar a AutoReady al vaciarse. */
    M00_EStop,				/**< Parada de emergencia. Corta potencia inmediatamente. */

    Status					/**< Solicita el estado actual de la FSM. */
};


/**
 * @struct CommandResult
 * @brief Empaqueta el resultado del procesamiento de un comando UART.
 */
struct CommandResult {
    bool handled;			/**< True si la FSM procesó el comando (incluso si falló). */
    bool success;			/**< True si la transición o acción fue exitosa. */
    char response[48];		/**< Buffer con el mensaje ACK/ERR a transmitir. */
};

/**
 * @struct PendingAutoCommand
 * @brief Estructura de Waypoint (Punto de trayectoria) parseado desde G-Code.
 */
struct PendingAutoCommand {
    bool valid;				/**< Bandera de integridad estructural del comando. */
    float q1;				/**< Consigna articular 1 (Base). */
    float q2;				/**< Consigna articular 2 (Hombro). */
    float q3;				/**< Consigna articular 3 (Codo). */
    uint8_t a1;				/**< Estado del actuador. */
};


/**
 * @class Controller
 * @brief Director de orquesta del sistema. Gestiona dependencias, FSM y comunicaciones.
 */
class Controller {
public:
	/**
	 * @brief Constructor mediante Inyección de Dependencias.
	 * @param terminal Enlace de comunicación UART.
	 * @param joystickIzquierdo Periférico de entrada analógica.
	 * @param servoBase Actuador principal del primer grado de libertad.
	 * @param efectorFinal Controlador del circuito neumático (Bomba y Válvula).
	 */
    Controller(TerminalSerial& terminal,
               Joystick& joystickIzquierdo, Joystick& joystickDerecho,
               Servo& servoBase, Servo& servoHombro, Servo& servoCodo,
			   Actuador& efectorFinal);

    /**
     * @brief Inicializa el estado del controlador y prepara la FSM.
     */
    void init(void);

    /**
     * @brief Bucle principal de resolución de la máquina de estados y tareas asíncronas.
     * @param nowMs Marca de tiempo del sistema (HAL_GetTick).
     */
    void update(uint32_t nowMs);

    // --- Getters de Estado ---
    GlobalState getState(void) const;
    GlobalState getPreviousState(void) const;
    const char* getStateName(void) const;
    static const char* stateToString(GlobalState state);

    // --- Gestión de Banderas de Transición ---
    bool didStateChange(void) const;
    void clearStateChangeFlag(void);

    // --- Forzado de Estados Críticos ---
    void forceFault(uint32_t nowMs);
    void forceEmergency(uint32_t nowMs);

private:
    // --- Dependencias Fisicas Inyectadas ---
    TerminalSerial& _terminal;

    Joystick& _joystickIzquierdo;
    Joystick& _joystickDerecho;

    Servo& _servoBase;
    Servo& _servoHombro;
    Servo& _servoCodo;

    Actuador& _efectorFinal;

    // --- Variables de Estado Interno del Controlador ---
    GlobalState _state;
    GlobalState _previousState;

    bool _stateChanged;
    bool _initialized;
    bool _primerIngresoIdle;

    uint32_t _stateEntryTick;
    uint32_t _tiempoUltimoEnvioTelemetria;
    uint32_t _tiempoInicioSuccion;			/**< Control del timeout de sujeción neumática. */

    /* ====================================================================
     * MEMORIA DINÁMICA: BUFFER CIRCULAR PARA TRAYECTORIAS (AUTO)
     * ==================================================================== */
    static constexpr uint8_t AUTO_FIFO_CAPACITY = 64U;
    PendingAutoCommand _autoFifo[AUTO_FIFO_CAPACITY];

    uint8_t _autoFifoHead;
    uint8_t _autoFifoTail;
    uint8_t _autoFifoCount;

    bool _chunkRequestPending;
    bool _endOfTrajectoryPending;			/**< true tras M116: drenar FIFO restante y pasar a AutoReady al vaciarse. */

    bool _homingActive;					/**< true mientras dura el regreso a Home disparado por el pulsador izquierdo (Manual/Recording). */

    // Control de Metrónomo Estricto 50Hz
    uint32_t _lastTrajectoryTick;

    // Metodos Utilitarios
    static const char* toString(GlobalState state);
    static const char* toString(UartCommand command);
    static void normalizeToUpperNoSpaces(const char* src, char* dst, uint16_t dstSize);
    static UartCommand parseUartCommand(const char* text);

    CommandResult makeAck(const char* ackText);
    CommandResult makeErr(const char* errText);
    CommandResult makeHandledNoReply(void);
    CommandResult makeNotHandled(void);
    bool tryParseGCommand(const char* rawLine, PendingAutoCommand& outCmd);

    // Gestion de Cola FIFO
    bool enqueueAutoPoint(const PendingAutoCommand& cmd);
    bool dequeueAutoPoint(PendingAutoCommand& cmd);
    bool isAutoFifoEmpty(void) const;
    bool isAutoFifoFull(void) const;
    void clearAutoFifo(void);
    void clearAutoRuntimeState(void);

    // Motor Principal de Trayectoria
    void requestNextChunkIfNeeded(void);
    void gestionarActuadorAuto(uint8_t a1, uint32_t nowMs);

    // Maquina de Estados
    void transitionTo(GlobalState newState, uint32_t nowMs);
    void onEnterState(GlobalState state);
    void runCurrentState(void);
    CommandResult handleUartLine(const char* rawLine, uint32_t nowMs);
    void processStateLocalCommand(const char* rawLine);

    // Rutinas de Accion
    void detenerActuadoresManuales(void);
    void realizarServoInitInicial(void);
    void realizarGoHomeNormal(void);
    void enviarTelemetria(void);
    void enviarEstadoActualPorUart(void);
    void procesarTeleoperacion(void);

    // Callbacks On-Enter
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

    // Callbacks On-Update
    void onUpdateInit(void);
    void onUpdateIdle(void);
    void onUpdateManual(void);
    void onUpdateRecording(void);
    void onUpdateAutoReady(void);
    void onUpdateAutoRunning(void);
    void onUpdateAutoPaused(void);
    void onUpdateFault(void);
    void onUpdateEmergency(void);

    // Callbacks locales
    void onManualCommand(const char* rawCommand);
    void onRecordingCommand(const char* rawCommand);
    void onAutoReadyCommand(const char* rawCommand);
    void onAutoRunningCommand(const char* rawCommand);
    void onAutoPausedCommand(const char* rawCommand);
    void onIdleCommand(const char* rawCommand);
};

} // namespace Station

#endif	/* STATION_CONTROLLER_HPP */
