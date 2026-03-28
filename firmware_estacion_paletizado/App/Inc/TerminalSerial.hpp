/**
 * @file TerminalSerial.hpp
 * @brief Capa de abstracción para telemetría y recepción de comandos no bloqueante.
 */

#ifndef TERMINALSERIAL_HPP
#define TERMINALSERIAL_HPP

#include <stdint.h>
#include <string.h>
#include <cstdio>
#include "stm32f1xx_hal.h"

// Tamaño máximo de los paquetes de datos
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 64

class TerminalSerial {
private:
    /* Enlace de hardware */
    UART_HandleTypeDef* _huart;

    /* Buffers de Memoria SRAM */
    char _txBuffer[TX_BUFFER_SIZE];
    char _rxBuffer[RX_BUFFER_SIZE];

    /* Variables de Estado para Recepción Asíncrona */
    uint8_t _rxByte; // Contenedor de 1 byte para la interrupción de hardware
    uint16_t _rxIndex;
    bool _comandoListo;

    void analizarComando();

public:
    /**
     * @brief Constructor de la terminal asíncrona.
     * @param huart Puntero a la estructura HAL del USART.
     * @param txIntervalMs Intervalo de envío automático de telemetría en milisegundos.
     */
    TerminalSerial(UART_HandleTypeDef* huart);


    void init();


    void update();

    /**
     * @brief Transmite una cadena de caracteres mediante DMA.
     * @param mensaje Puntero a la cadena a transmitir (terminada en null)
     * @ return true si el mensaje fue aceptado, false si el hardware estaba ocupado
     */
    bool enviarMensaje(const char* mensaje);


    /**
     * @brief Rutina de servicio llamada desde HAL_UART_RxCpltCallback.
     * @note Se ejecuta en contexto de interrupción (ISR). Tiempo de ejecución estricto O(1).
     */
    void interrupcionRecepcionByte();
};

#endif /* TERMINALSERIAL_HPP */
