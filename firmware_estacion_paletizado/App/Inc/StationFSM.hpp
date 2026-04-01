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
    Auto,
    Paused,
    Fault,
    EmergencyStop,
    Homing
};

const char* toString(GlobalState state);

typedef void (*StateCallback)(void);
typedef void (*StateCommandCallback)(const char* rawCommand);

struct Hooks {
    StateCallback onEnterBoot;
    StateCallback onEnterInit;
    StateCallback onEnterIdle;
    StateCallback onEnterManual;
    StateCallback onEnterAuto;
    StateCallback onEnterPaused;
    StateCallback onEnterFault;
    StateCallback onEnterEmergencyStop;
    StateCallback onEnterHoming;

    StateCallback onUpdateIdle;
    StateCallback onUpdateManual;
    StateCallback onUpdateAuto;
    StateCallback onUpdatePaused;
    StateCallback onUpdateFault;
    StateCallback onUpdateEmergencyStop;
    StateCallback onUpdateHoming;

    StateCommandCallback onManualCommand;
    StateCommandCallback onAutoCommand;
    StateCommandCallback onIdleCommand;
};

class FSM {
public:
    FSM();

    void init(void);
    void update(uint32_t nowMs);
    void handleUartLine(const char* rawLine);

    GlobalState getState(void) const;
    GlobalState getPreviousState(void) const;
    const char* getStateName(void) const;

    bool didStateChange(void) const;
    void clearStateChangeFlag(void);

    void setHooks(const Hooks& hooks);

private:
    GlobalState _state;
    GlobalState _previousState;
    GlobalState _stateBeforePause;
    bool _stateChanged;
    bool _initialized;
    uint32_t _stateEntryTick;
    Hooks _hooks;

    void transitionTo(GlobalState newState, uint32_t nowMs);
    void onEnterState(GlobalState state);
    void runCurrentState(void);
    void processModeCommand(UartCommand command, uint32_t nowMs);
    void processStateLocalCommand(const char* rawLine);
};

} // namespace Station

#endif /* STATIONFSM_HPP */
