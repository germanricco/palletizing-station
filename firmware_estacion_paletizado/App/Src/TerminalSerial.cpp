#include "TerminalSerial.hpp"

TerminalSerial::TerminalSerial(UART_HandleTypeDef* huart)
    : _huart(huart), _rxIndex(0), _comandoListo(false), _rxByte(0U) {
    memset(_txBuffer, 0, TX_BUFFER_SIZE);
    memset(_rxBuffer, 0, RX_BUFFER_SIZE);
}

void TerminalSerial::init() {
    HAL_UART_Receive_IT(_huart, &_rxByte, 1);
}

void TerminalSerial::update() {
    /*
     * Sin eco automático.
     * Solo arma líneas completas y las deja listas para main/FSM.
     */
}

bool TerminalSerial::enviarMensaje(const char* mensaje) {
    if (mensaje == nullptr) {
        return false;
    }

    if (_huart->gState != HAL_UART_STATE_READY) {
        return false;
    }

    strncpy(_txBuffer, mensaje, TX_BUFFER_SIZE - 1);
    _txBuffer[TX_BUFFER_SIZE - 1] = '\0';

    HAL_UART_Transmit_DMA(_huart, (uint8_t*)_txBuffer, strlen(_txBuffer));
    return true;
}

void TerminalSerial::interrupcionRecepcionByte() {
    if (_rxByte == '\r') {
        HAL_UART_Receive_IT(_huart, &_rxByte, 1);
        return;
    }

    if (_rxByte == '\n') {
        _rxBuffer[_rxIndex] = '\0';
        _comandoListo = true;
        return;
    }

    if (_rxIndex < (RX_BUFFER_SIZE - 1)) {
        _rxBuffer[_rxIndex++] = static_cast<char>(_rxByte);
    }

    HAL_UART_Receive_IT(_huart, &_rxByte, 1);
}

bool TerminalSerial::hayComandoPendiente() const {
    return _comandoListo;
}

const char* TerminalSerial::obtenerComandoPendiente() const {
    return _rxBuffer;
}

void TerminalSerial::limpiarComandoPendiente() {
    memset(_rxBuffer, 0, RX_BUFFER_SIZE);
    _rxIndex = 0;
    _comandoListo = false;
    HAL_UART_Receive_IT(_huart, &_rxByte, 1);
}
