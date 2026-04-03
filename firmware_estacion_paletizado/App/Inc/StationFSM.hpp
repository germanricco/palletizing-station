#ifndef STATIONFSM_HPP
#define STATIONFSM_HPP

#include <stdint.h>
#include "UartCommand.hpp"

namespace Station {

enum class GlobalState : uint8_t {
    Boot = 0,
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

const char* toString(GlobalState state);

typedef void (*StateCallback)(void);
typedef void (*StateCommandCallback)(const char* rawCommand);

struct CommandResult {
    bool handled;
    bool success;
    char response[48];
};

struct Hooks {
    StateCallback onEnterBoot;
    StateCallback onEnterInit;
    StateCallback onEnterIdle;
    StateCallback onEnterManual;
    StateCallback onEnterRecording;
    StateCallback onEnterAutoReady;
    StateCallback onEnterAutoRunning;
    StateCallback onEnterAutoPaused;
    StateCallback onEnterFault;
    StateCallback onEnterEmergency;

    StateCallback onUpdateIdle;
    StateCallback onUpdateManual;
    StateCallback onUpdateRecording;
    StateCallback onUpdateAutoReady;
    StateCallback onUpdateAutoRunning;
    StateCallback onUpdateAutoPaused;
    StateCallback onUpdateFault;
    StateCallback onUpdateEmergency;

    StateCommandCallback onManualCommand;
    StateCommandCallback onRecordingCommand;
    StateCommandCallback onAutoReadyCommand;
    StateCommandCallback onAutoRunningCommand;
    StateCommandCallback onAutoPausedCommand;
    StateCommandCallback onIdleCommand;
};

class FSM {
public:
    FSM();

    void init(void);
    void update(uint32_t nowMs);
    CommandResult handleUartLine(const char* rawLine, uint32_t nowMs);

    GlobalState getState(void) const;
    GlobalState getPreviousState(void) const;
    const char* getStateName(void) const;

    bool didStateChange(void) const;
    void clearStateChangeFlag(void);

    void setHooks(const Hooks& hooks);

    void forceFault(uint32_t nowMs);
    void forceEmergency(uint32_t nowMs);

private:
    GlobalState _state;
    GlobalState _previousState;
    bool _stateChanged;
    bool _initialized;
    uint32_t _stateEntryTick;
    Hooks _hooks;

    void transitionTo(GlobalState newState, uint32_t nowMs);
    void onEnterState(GlobalState state);
    void runCurrentState(void);
    void processStateLocalCommand(const char* rawLine);

    static CommandResult makeAck(const char* ackText);
    static CommandResult makeErr(const char* errText);
    static CommandResult makeHandledNoReply(void);
    static CommandResult makeNotHandled(void);
};

} // namespace Station

#endif /* STATIONFSM_HPP */
