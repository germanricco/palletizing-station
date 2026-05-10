#include "SensorIR.hpp"

SensorIR::SensorIR(GPIO_TypeDef* gpioPort,
                   uint16_t gpioPin,
                   GPIO_PinState estadoActivo,
                   uint32_t debounceMs)
    : _gpioPort(gpioPort),
      _gpioPin(gpioPin),
      _estadoActivo(estadoActivo),
      _debounceMs(debounceMs),
      _ultimaLecturaCruda(GPIO_PIN_SET),
      _lecturaEstable(GPIO_PIN_SET),
      _tiempoUltimoCambio(0),
      _interrupcionPendiente(false),
      _objetoDetectado(false)
{
}

void SensorIR::init(uint32_t tiempoActual)
{
    _ultimaLecturaCruda = leerPin();
    _lecturaEstable = _ultimaLecturaCruda;
    _tiempoUltimoCambio = tiempoActual;
    _objetoDetectado = (_lecturaEstable == _estadoActivo);
    _interrupcionPendiente = false;
}

bool SensorIR::update(uint32_t tiempoActual)
{
    GPIO_PinState lecturaActual = leerPin();

    if (lecturaActual != _ultimaLecturaCruda)
    {
        _ultimaLecturaCruda = lecturaActual;
        _tiempoUltimoCambio = tiempoActual;
        _interrupcionPendiente = true;
    }

    if (_interrupcionPendiente)
    {
        if ((tiempoActual - _tiempoUltimoCambio) >= _debounceMs)
        {
            if (_lecturaEstable != _ultimaLecturaCruda)
            {
                _lecturaEstable = _ultimaLecturaCruda;
                _objetoDetectado = (_lecturaEstable == _estadoActivo);
                _interrupcionPendiente = false;

                return true;
            }

            _interrupcionPendiente = false;
        }
    }

    return false;
}

void SensorIR::notificarInterrupcion(uint16_t gpioPin)
{
    if (pinCoincide(gpioPin))
    {
        _interrupcionPendiente = true;
    }
}

bool SensorIR::objetoDetectado() const
{
    return _objetoDetectado;
}

bool SensorIR::pinCoincide(uint16_t gpioPin) const
{
    return gpioPin == _gpioPin;
}

GPIO_PinState SensorIR::leerPin() const
{
    return HAL_GPIO_ReadPin(_gpioPort, _gpioPin);
}
