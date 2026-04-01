#include "StationFSM.hpp"
#include <cstring>

namespace Station {

static void emptyStateCallback(void) {}
static void emptyCommandCallback(const char*) {}

const char* toString(GlobalState state) {
    switch (state) {
        case GlobalState::Boot:          return "Boot";
        case GlobalState::Init:          return "Init";
        case GlobalState::Idle:          return "Idle";
        case GlobalState::Manual:        return "Manual";
        case GlobalState::Auto:          return "Auto";
        case GlobalState::Paused:        return "Paused";
        case GlobalState::Fault:         return "Fault";
        case GlobalState::EmergencyStop: return "EmergencyStop";
        case GlobalState::Homing:        return "Homing";
        default:                         return "UnknownState";
    }
}

FSM::FSM()
    : _state(GlobalState::Boot),
      _previousState(GlobalState::Boot),
      _stateBeforePause(GlobalState::Idle),
      _stateChanged(false),
      _initialized(false),
      _stateEntryTick(0U) {
    _hooks.onEnterBoot = emptyStateCallback;
    _hooks.onEnterInit = emptyStateCallback;
    _hooks.onEnterIdle = emptyStateCallback;
    _hooks.onEnterManual = emptyStateCallback;
    _hooks.onEnterAuto = emptyStateCallback;
    _hooks.onEnterPaused = emptyStateCallback;
    _hooks.onEnterFault = emptyStateCallback;
    _hooks.onEnterEmergencyStop = emptyStateCallback;
    _hooks.onEnterHoming = emptyStateCallback;

    _hooks.onUpdateIdle = emptyStateCallback;
    _hooks.onUpdateManual = emptyStateCallback;
    _hooks.onUpdateAuto = emptyStateCallback;
    _hooks.onUpdatePaused = emptyStateCallback;
    _hooks.onUpdateFault = emptyStateCallback;
    _hooks.onUpdateEmergencyStop = emptyStateCallback;
    _hooks.onUpdateHoming = emptyStateCallback;

    _hooks.onManualCommand = emptyCommandCallback;
    _hooks.onAutoCommand = emptyCommandCallback;
    _hooks.onIdleCommand = emptyCommandCallback;
}

void FSM::setHooks(const Hooks& hooks) {
    _hooks = hooks;
}

void FSM::init(void) {
    _state = GlobalState::Boot;
    _previousState = GlobalState::Boot;
    _stateBeforePause = GlobalState::Idle;
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

void FSM::handleUartLine(const char* rawLine) {
    if (rawLine == nullptr) {
        return;
    }

    const UartCommand command = parseUartCommand(rawLine);
    processModeCommand(command, _stateEntryTick);

    if ((command == UartCommand::Unknown) ||
        (command == UartCommand::None)) {
        processStateLocalCommand(rawLine);
        return;
    }

    if (command == UartCommand::Status) {
        return;
    }

    if ((command != UartCommand::Manual) &&
        (command != UartCommand::Auto) &&
        (command != UartCommand::Stop) &&
        (command != UartCommand::Pause) &&
        (command != UartCommand::Resume) &&
        (command != UartCommand::Reset) &&
        (command != UartCommand::EStop) &&
        (command != UartCommand::Home)) {
        processStateLocalCommand(rawLine);
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
        case GlobalState::Boot:          _hooks.onEnterBoot(); break;
        case GlobalState::Init:          _hooks.onEnterInit(); break;
        case GlobalState::Idle:          _hooks.onEnterIdle(); break;
        case GlobalState::Manual:        _hooks.onEnterManual(); break;
        case GlobalState::Auto:          _hooks.onEnterAuto(); break;
        case GlobalState::Paused:        _hooks.onEnterPaused(); break;
        case GlobalState::Fault:         _hooks.onEnterFault(); break;
        case GlobalState::EmergencyStop: _hooks.onEnterEmergencyStop(); break;
        case GlobalState::Homing:        _hooks.onEnterHoming(); break;
        default: break;
    }
}

void FSM::runCurrentState(void) {
    switch (_state) {
        case GlobalState::Idle:          _hooks.onUpdateIdle(); break;
        case GlobalState::Manual:        _hooks.onUpdateManual(); break;
        case GlobalState::Auto:          _hooks.onUpdateAuto(); break;
        case GlobalState::Paused:        _hooks.onUpdatePaused(); break;
        case GlobalState::Fault:         _hooks.onUpdateFault(); break;
        case GlobalState::EmergencyStop: _hooks.onUpdateEmergencyStop(); break;
        case GlobalState::Homing:        _hooks.onUpdateHoming(); break;
        default: break;
    }
}

void FSM::processModeCommand(UartCommand command, uint32_t nowMs) {
    switch (command) {
        case UartCommand::Manual:
            if ((_state == GlobalState::Idle) || (_state == GlobalState::Paused)) {
                transitionTo(GlobalState::Manual, nowMs);
            }
            break;

        case UartCommand::Auto:
            if ((_state == GlobalState::Idle) || (_state == GlobalState::Paused)) {
                transitionTo(GlobalState::Auto, nowMs);
            }
            break;

        case UartCommand::Home:
            if ((_state == GlobalState::Idle) || (_state == GlobalState::Manual)) {
                transitionTo(GlobalState::Homing, nowMs);
            }
            break;

        case UartCommand::Stop:
            if ((_state == GlobalState::Manual) ||
                (_state == GlobalState::Auto)   ||
                (_state == GlobalState::Homing) ||
                (_state == GlobalState::Paused)) {
                transitionTo(GlobalState::Idle, nowMs);
            }
            break;

        case UartCommand::Pause:
            if ((_state == GlobalState::Manual) ||
                (_state == GlobalState::Auto)   ||
                (_state == GlobalState::Homing)) {
                _stateBeforePause = _state;
                transitionTo(GlobalState::Paused, nowMs);
            }
            break;

        case UartCommand::Resume:
            if (_state == GlobalState::Paused) {
                transitionTo(_stateBeforePause, nowMs);
            }
            break;

        case UartCommand::Reset:
            if ((_state == GlobalState::Fault) ||
                (_state == GlobalState::EmergencyStop)) {
                transitionTo(GlobalState::Idle, nowMs);
            }
            break;

        case UartCommand::EStop:
            transitionTo(GlobalState::EmergencyStop, nowMs);
            break;

        default:
            break;
    }
}

void FSM::processStateLocalCommand(const char* rawLine) {
    switch (_state) {
        case GlobalState::Manual:
            _hooks.onManualCommand(rawLine);
            break;

        case GlobalState::Auto:
            _hooks.onAutoCommand(rawLine);
            break;

        case GlobalState::Idle:
            _hooks.onIdleCommand(rawLine);
            break;

        default:
            break;
    }
}

} // namespace Station
