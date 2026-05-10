#ifndef TERMINALSERIAL_HPP
#define TERMINALSERIAL_HPP

#include "stm32f1xx_hal.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

/*
 * ================================================================
 * TERMINAL SERIAL
 * ================================================================
 *
 * Responsabilidades:
 * 1) Recibir comandos UART byte a byte por interrupción.
 * 2) Armar líneas completas terminadas en '\n'.
 * 3) Guardar varias líneas completas en una cola RX circular.
 * 4) Enviar mensajes por UART usando DMA con colas de prioridad.
 *
 * Por qué existe la cola RX:
 * - Durante streaming, Python puede enviar varias líneas G1 seguidas.
 * - Si el firmware guarda una sola línea pendiente, las siguientes se pierden
 *   antes de que el loop principal alcance a procesarlas.
 * - Esta cola evita ese colapso lógico del protocolo.
 * ================================================================
 */
class TerminalSerial {
public:
    enum class TxPriority : uint8_t {
        HIGH = 0,
        NORMAL,
        LOW,
        NONE
    };

    struct TxSlot {
        char data[128];
        uint16_t len;
    };

public:
    explicit TerminalSerial(UART_HandleTypeDef* huart);

    void init();
    void update();

    bool enviarACK(const char* msg);
    bool enviarNormal(const char* msg);
    bool enviarLog(const char* msg);
    bool enviarMensaje(const char* msg);

    void onTxCompleteISR();
    void onTxErrorISR();
    void onRxErrorISR();
    void interrupcionRecepcionByte();

    bool hayComandoPendiente() const;
    const char* obtenerComandoPendiente() const;
    void limpiarComandoPendiente();

private:
    static constexpr uint16_t RX_LINE_SIZE = 128;
    static constexpr uint8_t  RX_QUEUE_LEN = 48;

    static constexpr uint16_t TX_BUFFER_SIZE = 128;
    static constexpr uint8_t  TX_QUEUE_LEN   = 8;

    UART_HandleTypeDef* _huart;

    /* RX byte a byte + cola de líneas completas */
    volatile uint8_t _rxByte;
    char _rxBuildBuffer[RX_LINE_SIZE];
    volatile uint16_t _rxIndex;
    volatile bool _rxOverflow;
    volatile bool _rxQueueOverflow;

    char _rxQueue[RX_QUEUE_LEN][RX_LINE_SIZE];
    volatile uint8_t _rxHead;
    volatile uint8_t _rxTail;
    volatile uint8_t _rxCount;

    /* TX por prioridad */
    TxSlot _qHigh[TX_QUEUE_LEN];
    TxSlot _qNormal[TX_QUEUE_LEN];
    TxSlot _qLow[TX_QUEUE_LEN];

    volatile uint8_t _hH, _tH, _cH;
    volatile uint8_t _hN, _tN, _cN;
    volatile uint8_t _hL, _tL, _cL;

    volatile bool _txBusy;
    volatile TxPriority _txActivePriority;

private:
    bool enqueue(TxPriority prio, const char* msg);
    bool iniciarTx();
    void popActiveQueue();

    uint32_t enterCritical();
    void exitCritical(uint32_t primask);
};

#endif
