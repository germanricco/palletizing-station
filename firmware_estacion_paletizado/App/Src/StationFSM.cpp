#include "StationFSM.hpp"
#include <cstring>
#include <cstdio>

namespace Station {

static void emptyStateCallback(void) {}
static void emptyCommandCallback(const char*) {}

const char* toString(GlobalState state) {
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

FSM::FSM()
    : _state(GlobalState::Boot),
      _previousState(GlobalState::Boot),
      _stateChanged(false),
      _initialized(false),
      _stateEntryTick(0U) {
    _hooks.onEnterBoot        = emptyStateCallback;
    _hooks.onEnterInit        = emptyStateCallback;
    _hooks.onEnterIdle        = emptyStateCallback;
    _hooks.onEnterManual      = emptyStateCallback;
    _hooks.onEnterRecording   = emptyStateCallback;
    _hooks.onEnterAutoReady   = emptyStateCallback;
    _hooks.onEnterAutoRunning = emptyStateCallback;
    _hooks.onEnterAutoPaused  = emptyStateCallback;
    _hooks.onEnterFault       = emptyStateCallback;
    _hooks.onEnterEmergency   = emptyStateCallback;

    _hooks.onUpdateIdle        = emptyStateCallback;
    _hooks.onUpdateManual      = emptyStateCallback;
    _hooks.onUpdateRecording   = emptyStateCallback;
    _hooks.onUpdateAutoReady   = emptyStateCallback;
    _hooks.onUpdateAutoRunning = emptyStateCallback;
    _hooks.onUpdateAutoPaused  = emptyStateCallback;
    _hooks.onUpdateFault       = emptyStateCallback;
    _hooks.onUpdateEmergency   = emptyStateCallback;

    _hooks.onManualCommand      = emptyCommandCallback;
    _hooks.onRecordingCommand   = emptyCommandCallback;
    _hooks.onAutoReadyCommand   = emptyCommandCallback;
    _hooks.onAutoRunningCommand = emptyCommandCallback;
    _hooks.onAutoPausedCommand  = emptyCommandCallback;
    _hooks.onIdleCommand        = emptyCommandCallback;
}

void FSM::setHooks(const Hooks& hooks) {
    _hooks = hooks;
}

void FSM::init(void) {
    _state = GlobalState::Boot;
    _previousState = GlobalState::Boot;
    _stateChanged = true;
    _initialized = true;
    _stateEntryTick = 0U;
    onEnterState(_state);
}

void FSM::update(uint32_t nowMs) {
    if (!_initialized) {
        init();
    }

    switch (_state) {
        case GlobalState::Boot:
            transitionTo(GlobalState::Init, nowMs);
            break;

        case GlobalState::Init:
            transitionTo(GlobalState::Idle, nowMs);
            break;

        default:
            break;
    }

    runCurrentState();
}

CommandResult FSM::makeAck(const char* ackText) {
    CommandResult result{};
    result.handled = true;
    result.success = true;
    std::snprintf(result.response, sizeof(result.response), "%s\r\n", ackText);
    return result;
}

CommandResult FSM::makeErr(const char* errText) {
    CommandResult result{};
    result.handled = true;
    result.success = false;
    std::snprintf(result.response, sizeof(result.response), "%s\r\n", errText);
    return result;
}

CommandResult FSM::makeHandledNoReply(void) {
    CommandResult result{};
    result.handled = true;
    result.success = true;
    result.response[0] = '\0';
    return result;
}

CommandResult FSM::makeNotHandled(void) {
    CommandResult result{};
    result.handled = false;
    result.success = false;
    result.response[0] = '\0';
    return result;
}

CommandResult FSM::handleUartLine(const char* rawLine, uint32_t nowMs) {
    if (rawLine == nullptr) {
        return makeErr("ERR:NULL_CMD");
    }

    const UartCommand command = parseUartCommand(rawLine);

    /*
     * PRIORIDAD ABSOLUTA
     */
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
        case GlobalState::Idle:
            if (command == UartCommand::M100_ManualMode) {
                transitionTo(GlobalState::Manual, nowMs);
                return makeAck("ACK_M100");
            }
            if (command == UartCommand::M101_AutoMode) {
                transitionTo(GlobalState::AutoReady, nowMs);
                return makeAck("ACK_M101");
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
        case GlobalState::Init:
            return makeErr("ERR:NOT_READY");

        default:
            return makeErr("ERR:UNEXPECTED_STATE");
    }
}

GlobalState FSM::getState(void) const {
    return _state;
}

GlobalState FSM::getPreviousState(void) const {
    return _previousState;
}

const char* FSM::getStateName(void) const {
    return toString(_state);
}

bool FSM::didStateChange(void) const {
    return _stateChanged;
}

void FSM::clearStateChangeFlag(void) {
    _stateChanged = false;
}

void FSM::transitionTo(GlobalState newState, uint32_t nowMs) {
    if (_state == newState) {
        return;
    }

    _previousState = _state;
    _state = newState;
    _stateEntryTick = nowMs;
    _stateChanged = true;
    onEnterState(_state);
}

void FSM::onEnterState(GlobalState state) {
    switch (state) {
        case GlobalState::Boot:        _hooks.onEnterBoot(); break;
        case GlobalState::Init:        _hooks.onEnterInit(); break;
        case GlobalState::Idle:        _hooks.onEnterIdle(); break;
        case GlobalState::Manual:      _hooks.onEnterManual(); break;
        case GlobalState::Recording:   _hooks.onEnterRecording(); break;
        case GlobalState::AutoReady:   _hooks.onEnterAutoReady(); break;
        case GlobalState::AutoRunning: _hooks.onEnterAutoRunning(); break;
        case GlobalState::AutoPaused:  _hooks.onEnterAutoPaused(); break;
        case GlobalState::Fault:       _hooks.onEnterFault(); break;
        case GlobalState::Emergency:   _hooks.onEnterEmergency(); break;
        default: break;
    }
}

void FSM::runCurrentState(void) {
    switch (_state) {
        case GlobalState::Idle:        _hooks.onUpdateIdle(); break;
        case GlobalState::Manual:      _hooks.onUpdateManual(); break;
        case GlobalState::Recording:   _hooks.onUpdateRecording(); break;
        case GlobalState::AutoReady:   _hooks.onUpdateAutoReady(); break;
        case GlobalState::AutoRunning: _hooks.onUpdateAutoRunning(); break;
        case GlobalState::AutoPaused:  _hooks.onUpdateAutoPaused(); break;
        case GlobalState::Fault:       _hooks.onUpdateFault(); break;
        case GlobalState::Emergency:   _hooks.onUpdateEmergency(); break;
        default: break;
    }
}

void FSM::processStateLocalCommand(const char* rawLine) {
    switch (_state) {
        case GlobalState::Manual:
            _hooks.onManualCommand(rawLine);
            break;

        case GlobalState::Recording:
            _hooks.onRecordingCommand(rawLine);
            break;

        case GlobalState::AutoReady:
            _hooks.onAutoReadyCommand(rawLine);
            break;

        case GlobalState::AutoRunning:
            _hooks.onAutoRunningCommand(rawLine);
            break;

        case GlobalState::AutoPaused:
            _hooks.onAutoPausedCommand(rawLine);
            break;

        case GlobalState::Idle:
            _hooks.onIdleCommand(rawLine);
            break;

        default:
            break;
    }
}

void FSM::forceFault(uint32_t nowMs) {
    transitionTo(GlobalState::Fault, nowMs);
}

void FSM::forceEmergency(uint32_t nowMs) {
    transitionTo(GlobalState::Emergency, nowMs);
}

} // namespace Station
