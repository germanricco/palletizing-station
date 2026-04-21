/**
 * @file Actuador.cpp
 * @brief Implementación de la máquina de estados del efector final neumático.
 */

#include "Actuador.hpp"

Actuador::Actuador(volatile uint32_t* ccr_pump, volatile uint32_t* ccr_valve,
                   uint32_t arr_max, uint32_t release_duration_ms)
    : _ccr_pump(ccr_pump),
      _ccr_valve(ccr_valve),
      _arr_max(arr_max),
      _currentState(ActuatorState::Idle),
      _release_start_time_ms(0),
      _release_duration_ms(release_duration_ms) {}

void Actuador::init() {
    commandIdle();
}

void Actuador::setPumpDutyCycle(float normalized_duty) {
    if (_ccr_pump == nullptr) return;

    // Saturación
    if (normalized_duty > 1.0f) normalized_duty = 1.0f;
    else if (normalized_duty < 0.0f) normalized_duty = 0.0f;

    // Ejecución
    *_ccr_pump = static_cast<uint32_t>(normalized_duty * static_cast<float>(_arr_max));
}

void Actuador::setValveDutyCycle(float normalized_duty) {
    if (_ccr_valve == nullptr) return;

    // Saturacion
    if (normalized_duty > 1.0f) normalized_duty = 1.0f;
    else if (normalized_duty < 0.0f) normalized_duty = 0.0f;

    // Ejecucion
    *_ccr_valve = static_cast<uint32_t>(normalized_duty * static_cast<float>(_arr_max));
}

void Actuador::commandSuction(float power) {
    _currentState = ActuatorState::Suction;
    setValveDutyCycle(0.0f); // Electroválvula mecánicamente cerrada
    setPumpDutyCycle(power); // Bomba energizada al nivel requerido
}

void Actuador::commandHold(float maintenance_power) {
    _currentState = ActuatorState::Holding;
    setValveDutyCycle(0.0f);
    setPumpDutyCycle(maintenance_power); // Generalmente 0.0f, >0 si el sistema tiene microfugas
}

void Actuador::commandRelease() {
    _currentState = ActuatorState::Releasing;
    setPumpDutyCycle(0.0f);

    // Apertura total de la electroválvula para máxima velocidad de ecualización
    setValveDutyCycle(1.0f);

    // El timestamp se evaluará asíncronamente en el método update()
    // Nota: El HAL_GetTick() se pasará desde el bucle principal en la próxima iteración.
    _release_start_time_ms = 0; // Se marca como pendiente de inicio
}

void Actuador::commandIdle() {
    _currentState = ActuatorState::Idle;
    setPumpDutyCycle(0.0f);
    setValveDutyCycle(0.0f);
}

void Actuador::update(uint32_t current_time_ms) {
    // Resolución de la máquina de estados temporal
    if (_currentState == ActuatorState::Releasing) {

        // Inicialización de la marca de tiempo diferida (Evita llamadas a HAL_GetTick internamente)
        if (_release_start_time_ms == 0) {
            _release_start_time_ms = current_time_ms;
        }

        // Transición asíncrona (Reemplaza el delay bloqueante del fabricante)
        if ((current_time_ms - _release_start_time_ms) >= _release_duration_ms) {
            commandIdle();
        }
    }
}

ActuatorState Actuador::getState() const {
    return _currentState;
}
