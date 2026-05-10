#ifndef SENSOR_IR_HPP
#define SENSOR_IR_HPP

#include "stm32f1xx_hal.h"
#include <cstdint>

class SensorIR {
public:
    SensorIR(GPIO_TypeDef* gpioPort,
             uint16_t gpioPin,
             GPIO_PinState estadoActivo = GPIO_PIN_RESET,
             uint32_t debounceMs = 30);

    void init(uint32_t tiempoActual);

    bool update(uint32_t tiempoActual);

    void notificarInterrupcion(uint16_t gpioPin);

    bool objetoDetectado() const;

    bool pinCoincide(uint16_t gpioPin) const;

private:
    GPIO_PinState leerPin() const;

private:
    GPIO_TypeDef* _gpioPort;
    uint16_t _gpioPin;

    GPIO_PinState _estadoActivo;
    uint32_t _debounceMs;

    GPIO_PinState _ultimaLecturaCruda;
    GPIO_PinState _lecturaEstable;

    uint32_t _tiempoUltimoCambio;

    volatile bool _interrupcionPendiente;
    bool _objetoDetectado;
};

#endif
