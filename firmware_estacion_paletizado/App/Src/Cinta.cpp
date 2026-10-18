/**
 * @file Cinta.cpp
 * @brief Implementacion de la sub-maquina de estados de la cinta transportadora.
 */

#include "Cinta.hpp"
#include "Logger.hpp"

Cinta::Cinta(MotorDC& motor, SensorIR& sensor, float velocidadNominal, uint32_t rampaArranqueMs)
    : _motor(motor),
      _sensor(sensor),
      _velocidadNominal(velocidadNominal),
      _rampaArranqueMs(rampaArranqueMs),
      _estado(EstadoCinta::Deshabilitada),
      _velocidadObjetivo(0.0f),
      _velocidadInicioRampa(0.0f),
      _velocidadAplicada(0.0f),
      _inicioRampaMs(0U),
      _pickLiberado(false) {

    // Saturacion de la consigna nominal
    if (_velocidadNominal > 1.0f) _velocidadNominal = 1.0f;
    else if (_velocidadNominal < 0.0f) _velocidadNominal = 0.0f;
}

void Cinta::init(uint32_t nowMs) {
    _estado = EstadoCinta::Deshabilitada;
    _pickLiberado = false;
    _inicioRampaMs = nowMs;

    // Fuerza la escritura al hardware aunque _velocidadAplicada ya valga 0
    _motor.setSpeed(0.0f);
    _velocidadAplicada = 0.0f;
    _velocidadObjetivo = 0.0f;
    _velocidadInicioRampa = 0.0f;
}

void Cinta::iniciarRampa(float objetivo, uint32_t nowMs) {
    _velocidadObjetivo = objetivo;
    _inicioRampaMs = nowMs;

    if (objetivo <= _velocidadAplicada) {
        // Sin rampa de bajada: la reduccion de velocidad se aplica de inmediato
        _velocidadInicioRampa = objetivo;
        aplicarVelocidad(objetivo);
    } else {
        _velocidadInicioRampa = _velocidadAplicada;
    }
}

void Cinta::detenerInmediato() {
    _velocidadObjetivo = 0.0f;
    _velocidadInicioRampa = 0.0f;
    aplicarVelocidad(0.0f);
}

void Cinta::aplicarVelocidad(float velocidad) {
    if (velocidad == _velocidadAplicada) return;

    _velocidadAplicada = velocidad;
    _motor.setSpeed(velocidad);
}

EventoCinta Cinta::update(uint32_t nowMs) {
    EventoCinta evento = EventoCinta::Ninguno;

    // 1. Sensor (antirrebote). Retorna true solo en un cambio estable de nivel.
    const bool cambioSensor = _sensor.update(nowMs);

    // 2. Flanco de deteccion: frenado inmediato
    if (cambioSensor && _sensor.objetoDetectado()) {
        if (_estado == EstadoCinta::Avanzando) {
            detenerInmediato();
            _estado = EstadoCinta::EsperandoPick;
            evento = EventoCinta::ObjetoDetectado;
            Logger::logDEBUG("CINTA: Objeto detectado. Detenida esperando pick.");
        } else if (_estado == EstadoCinta::Manual) {
            // Protege contra empujar la caja fuera del extremo de la cinta
            detenerInmediato();
            _estado = EstadoCinta::Deshabilitada;
            Logger::logINFO("CINTA: Marcha manual detenida por sensor.");
        }
    }

    // 3. Rearranque: pick liberado por el orquestador y zona del sensor libre
    if ((_estado == EstadoCinta::EsperandoPick) && _pickLiberado && !_sensor.objetoDetectado()) {
        _pickLiberado = false;
        iniciarRampa(_velocidadNominal, nowMs);
        _estado = EstadoCinta::Avanzando;
        Logger::logDEBUG("CINTA: Pick liberado y sensor libre. Reanudando avance.");
    }

    // 4. Generador de rampa (basado en tiempo; la resta uint32_t tolera el overflow de HAL_GetTick)
    if ((_estado == EstadoCinta::Avanzando) || (_estado == EstadoCinta::Manual)) {
        const uint32_t transcurrido = nowMs - _inicioRampaMs;

        if ((_rampaArranqueMs == 0U) || (transcurrido >= _rampaArranqueMs)) {
            aplicarVelocidad(_velocidadObjetivo);
        } else {
            const float fraccion = static_cast<float>(transcurrido) / static_cast<float>(_rampaArranqueMs);
            aplicarVelocidad(_velocidadInicioRampa + ((_velocidadObjetivo - _velocidadInicioRampa) * fraccion));
        }
    }

    return evento;
}

EventoCinta Cinta::armar(uint32_t nowMs) {
    // Idempotente: rearmar no debe reiniciar un ciclo en curso
    if ((_estado == EstadoCinta::Avanzando) || (_estado == EstadoCinta::EsperandoPick)) {
        return EventoCinta::Ninguno;
    }

    _pickLiberado = false;

    if (_sensor.objetoDetectado()) {
        // Caja ya presente al armar: se informa de inmediato (el flanco ya ocurrio)
        detenerInmediato();
        _estado = EstadoCinta::EsperandoPick;
        Logger::logDEBUG("CINTA: Armada con objeto presente. Esperando pick.");
        return EventoCinta::ObjetoDetectado;
    }

    iniciarRampa(_velocidadNominal, nowMs);
    _estado = EstadoCinta::Avanzando;
    Logger::logDEBUG("CINTA: Armada. Iniciando avance.");
    return EventoCinta::Ninguno;
}

void Cinta::desarmar() {
    detenerInmediato();
    _pickLiberado = false;

    if (_estado != EstadoCinta::Deshabilitada) {
        _estado = EstadoCinta::Deshabilitada;
        Logger::logDEBUG("CINTA: Deshabilitada.");
    }
}

void Cinta::liberarPick() {
    if (_estado == EstadoCinta::EsperandoPick) {
        _pickLiberado = true;
    }
}

bool Cinta::setVelocidadManual(float velocidad, uint32_t nowMs) {
    // Saturacion
    if (velocidad > 1.0f) velocidad = 1.0f;
    else if (velocidad < 0.0f) velocidad = 0.0f;

    if (velocidad == 0.0f) {
        desarmar();
        return true;
    }

    if (_sensor.objetoDetectado()) {
        Logger::logINFO("CINTA: Marcha manual rechazada. Objeto frente al sensor.");
        return false;
    }

    iniciarRampa(velocidad, nowMs);
    _pickLiberado = false;
    _estado = EstadoCinta::Manual;
    return true;
}

EstadoCinta Cinta::getEstado() const {
    return _estado;
}

bool Cinta::objetoPresente() const {
    return _sensor.objetoDetectado();
}

const char* Cinta::toString(EstadoCinta estado) {
    switch (estado) {
        case EstadoCinta::Deshabilitada: return "DESHABILITADA";
        case EstadoCinta::Avanzando:     return "AVANZANDO";
        case EstadoCinta::EsperandoPick: return "ESPERANDO_PICK";
        case EstadoCinta::Manual:        return "MANUAL";
        default:                         return "DESCONOCIDO";
    }
}
