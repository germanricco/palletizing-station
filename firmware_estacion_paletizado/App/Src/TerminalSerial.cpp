#include "TerminalSerial.hpp"

/**
 * Inicializacion de params internos y buffers de memoria en 0.
 */
TerminalSerial::TerminalSerial(UART_HandleTypeDef* huart)
    : _huart(huart), _rxIndex(0), _comandoListo(false) {
    memset(_txBuffer, 0, TX_BUFFER_SIZE);
    memset(_rxBuffer, 0, RX_BUFFER_SIZE);
}

void TerminalSerial::init() {
	// Activa interrupciones de UART
    HAL_UART_Receive_IT(_huart, &_rxByte, 1);
}

void TerminalSerial::update() {
    if (_comandoListo) {
        analizarComando();

        memset(_rxBuffer, 0, RX_BUFFER_SIZE);
        _rxIndex = 0;
        _comandoListo = false;

        HAL_UART_Receive_IT(_huart, &_rxByte, 1);
    }
}

bool TerminalSerial::enviarMensaje(const char* mensaje) {
    // 1. Barrera de Seguridad: Verificar si el periférico o el DMA están transmitiendo
    if (_huart->gState != HAL_UART_STATE_READY) {
        return false; // Hardware ocupado, se descarta la solicitud
    }

    // 2. Aislamiento de Memoria: Copiar a un buffer persistente
    // strncpy previene ataques o errores por desbordamiento de memoria (Buffer Overflow)
    strncpy(_txBuffer, mensaje, TX_BUFFER_SIZE - 1);
    _txBuffer[TX_BUFFER_SIZE - 1] = '\0'; // Garantizar terminador nulo

    // 3. Ejecución No Bloqueante
    HAL_UART_Transmit_DMA(_huart, (uint8_t*)_txBuffer, strlen(_txBuffer));
    return true;
}

void TerminalSerial::analizarComando() {
	// 1. Preparar un contenedor temporal para armar la respuesta
	// RX_BUFFER_SIZE es 64, por lo que 100 bytes es seguro para incluir el prefijo
	char respuesta_eco[100];

	// 2. Formatear el mensaje
	// Recordar: Nuestra interrupción reemplazó el '\n' entrante por un '\0',
	// así que _rxBuffer contiene el texto puro ("Hola"). Le agregamos \r\n al final.
	sprintf(respuesta_eco, "ECO: %s\r\n", _rxBuffer);

	// 3. Despachar el mensaje al DMA
	enviarMensaje(respuesta_eco);
}

void TerminalSerial::interrupcionRecepcionByte() {
    if (_rxByte == '\r') {
        HAL_UART_Receive_IT(_huart, &_rxByte, 1);
        return;
    }

    if (_rxByte == '\n') {
        _rxBuffer[_rxIndex] = '\0';
        _comandoListo = true;
    } else {
        if (_rxIndex < (RX_BUFFER_SIZE - 1)) {
            _rxBuffer[_rxIndex++] = (char)_rxByte;
        }
        HAL_UART_Receive_IT(_huart, &_rxByte, 1);
    }
}
