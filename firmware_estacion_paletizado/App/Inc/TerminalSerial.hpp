#ifndef TERMINALSERIAL_HPP
#define TERMINALSERIAL_HPP

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <cstring>

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 64

class TerminalSerial {
public:
    enum class TxPriority : uint8_t {
        HIGH = 0,   // ACK
        NORMAL,     // Telemetría
        LOW         // Logs
    };

private:
    UART_HandleTypeDef* _huart;

    // RX
    char _rxBuffer[RX_BUFFER_SIZE];
    uint8_t _rxByte;
    uint16_t _rxIndex;
    volatile bool _comandoListo;

    // TX
    static constexpr uint8_t TX_QUEUE_LEN = 6;

    struct TxSlot {
        char data[TX_BUFFER_SIZE];
        uint16_t len;
    };

    // 3 colas
    TxSlot _qHigh[TX_QUEUE_LEN];
    TxSlot _qNormal[TX_QUEUE_LEN];
    TxSlot _qLow[TX_QUEUE_LEN];

    volatile uint8_t _hH, _tH, _cH;
    volatile uint8_t _hN, _tN, _cN;
    volatile uint8_t _hL, _tL, _cL;

    volatile bool _txBusy;

    // internos
    bool iniciarTx();
    static uint32_t enterCritical();
    static void exitCritical(uint32_t);

    bool enqueue(TxPriority prio, const char* msg);

public:
    explicit TerminalSerial(UART_HandleTypeDef* huart);

    void init();
    void update();

    // API
    bool enviarACK(const char* msg);
    bool enviarNormal(const char* msg);
    bool enviarLog(const char* msg);
    bool enviarMensaje(const char* msg);

    // RX
    void interrupcionRecepcionByte();
    bool hayComandoPendiente() const;
    const char* obtenerComandoPendiente() const;
    void limpiarComandoPendiente();

    // ISR
    void onTxCompleteISR();
    void onTxErrorISR();
};

#endif
