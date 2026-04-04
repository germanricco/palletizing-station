#include "TerminalSerial.hpp"
#include <cstdio>

TerminalSerial::TerminalSerial(UART_HandleTypeDef* huart)
    : _huart(huart),
      _rxIndex(0),
      _comandoListo(false),
      _hH(0), _tH(0), _cH(0),
      _hN(0), _tN(0), _cN(0),
      _hL(0), _tL(0), _cL(0),
      _txBusy(false) {}

void TerminalSerial::init() {
    HAL_UART_Receive_IT(_huart, &_rxByte, 1);
}

void TerminalSerial::update() {}

uint32_t TerminalSerial::enterCritical() {
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}

void TerminalSerial::exitCritical(uint32_t p) {
    if (p == 0) __enable_irq();
}

bool TerminalSerial::enqueue(TxPriority prio, const char* msg) {
    size_t len = strnlen(msg, TX_BUFFER_SIZE - 1);

    TxSlot* q;
    volatile uint8_t *h, *t, *c;

    switch (prio) {
        case TxPriority::HIGH:   q=_qHigh; h=&_hH; t=&_tH; c=&_cH; break;
        case TxPriority::NORMAL: q=_qNormal; h=&_hN; t=&_tN; c=&_cN; break;
        default:                 q=_qLow; h=&_hL; t=&_tL; c=&_cL; break;
    }

    if (*c >= TX_QUEUE_LEN) return false;

    TxSlot& s = q[*t];
    memcpy(s.data, msg, len);
    s.data[len] = '\0';
    s.len = len;

    *t = (*t + 1) % TX_QUEUE_LEN;
    (*c)++;

    return true;
}

bool TerminalSerial::iniciarTx() {
    if (_txBusy) return false;

    TxSlot* s = nullptr;

    if (_cH) s = &_qHigh[_hH];
    else if (_cN) s = &_qNormal[_hN];
    else if (_cL) s = &_qLow[_hL];
    else return false;

    if (HAL_UART_Transmit_DMA(_huart, (uint8_t*)s->data, s->len) == HAL_OK) {
        _txBusy = true;
        return true;
    }

    return false;
}

bool TerminalSerial::enviarACK(const char* msg) {
    uint32_t p = enterCritical();
    enqueue(TxPriority::HIGH, msg);
    if (!_txBusy) iniciarTx();
    exitCritical(p);
    return true;
}

bool TerminalSerial::enviarNormal(const char* msg) {
    uint32_t p = enterCritical();
    enqueue(TxPriority::NORMAL, msg);
    if (!_txBusy) iniciarTx();
    exitCritical(p);
    return true;
}

bool TerminalSerial::enviarLog(const char* msg) {
    uint32_t p = enterCritical();
    enqueue(TxPriority::LOW, msg);
    if (!_txBusy) iniciarTx();
    exitCritical(p);
    return true;
}

bool TerminalSerial::enviarMensaje(const char* msg) {
    return enviarNormal(msg);
}

void TerminalSerial::onTxCompleteISR() {
    uint32_t p = enterCritical();

    if (_cH) { _hH = (_hH+1)%TX_QUEUE_LEN; _cH--; }
    else if (_cN) { _hN = (_hN+1)%TX_QUEUE_LEN; _cN--; }
    else if (_cL) { _hL = (_hL+1)%TX_QUEUE_LEN; _cL--; }

    _txBusy = false;

    iniciarTx();

    exitCritical(p);
}

void TerminalSerial::onTxErrorISR() {
    _txBusy = false;
}

void TerminalSerial::interrupcionRecepcionByte() {
    if (_rxByte == '\n') {
        _rxBuffer[_rxIndex] = '\0';
        _comandoListo = true;
    } else if (_rxIndex < RX_BUFFER_SIZE-1) {
        _rxBuffer[_rxIndex++] = _rxByte;
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
}
