#ifndef TERMINALSERIAL_HPP
#define TERMINALSERIAL_HPP

#include <stdint.h>
#include <string.h>
#include <cstdio>
#include "stm32f1xx_hal.h"

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 64

class TerminalSerial {
private:
    UART_HandleTypeDef* _huart;

    char _txBuffer[TX_BUFFER_SIZE];
    char _rxBuffer[RX_BUFFER_SIZE];

    uint8_t _rxByte;
    uint16_t _rxIndex;
    bool _comandoListo;

public:
    TerminalSerial(UART_HandleTypeDef* huart);

    void init();
    void update();
    bool enviarMensaje(const char* mensaje);
    void interrupcionRecepcionByte();

    bool hayComandoPendiente() const;
    const char* obtenerComandoPendiente() const;
    void limpiarComandoPendiente();
};

#endif /* TERMINALSERIAL_HPP */
