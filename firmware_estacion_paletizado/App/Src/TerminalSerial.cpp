#include "TerminalSerial.hpp"
#include <cstdio>
#include <cstring>

TerminalSerial::TerminalSerial(UART_HandleTypeDef* huart)
    : _huart(huart),
      _rxByte(0),
      _rxBuildBuffer{0},
      _rxIndex(0),
      _rxOverflow(false),
      _rxQueueOverflow(false),
      _rxQueue{},
      _rxHead(0),
      _rxTail(0),
      _rxCount(0),
      _qHigh{},
      _qNormal{},
      _qLow{},
      _hH(0), _tH(0), _cH(0),
      _hN(0), _tN(0), _cN(0),
      _hL(0), _tL(0), _cL(0),
      _txBusy(false),
      _txActivePriority(TxPriority::NONE) {
}

void TerminalSerial::init() {
    HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);
}

void TerminalSerial::update() {
}

uint32_t TerminalSerial::enterCritical() {
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}

void TerminalSerial::exitCritical(uint32_t primask) {
    if (primask == 0U) {
        __enable_irq();
    }
}

bool TerminalSerial::enqueue(TxPriority prio, const char* msg) {
    if (msg == nullptr) {
        return false;
    }

    const size_t len = strnlen(msg, TX_BUFFER_SIZE - 1U);

    TxSlot* q = nullptr;
    volatile uint8_t* t = nullptr;
    volatile uint8_t* c = nullptr;

    switch (prio) {
        case TxPriority::HIGH:
            q = _qHigh;
            t = &_tH;
            c = &_cH;
            break;

        case TxPriority::NORMAL:
            q = _qNormal;
            t = &_tN;
            c = &_cN;
            break;

        case TxPriority::LOW:
            q = _qLow;
            t = &_tL;
            c = &_cL;
            break;

        default:
            return false;
    }

    if (*c >= TX_QUEUE_LEN) {
        return false;
    }

    TxSlot& s = q[*t];
    memcpy(s.data, msg, len);
    s.data[len] = '\0';
    s.len = static_cast<uint16_t>(len);

    *t = static_cast<uint8_t>((*t + 1U) % TX_QUEUE_LEN);
    (*c)++;

    return true;
}

bool TerminalSerial::iniciarTx() {
    if (_txBusy) {
        return false;
    }

    TxSlot* slot = nullptr;
    TxPriority prio = TxPriority::NONE;

    if (_cH > 0U) {
        slot = &_qHigh[_hH];
        prio = TxPriority::HIGH;
    } else if (_cN > 0U) {
        slot = &_qNormal[_hN];
        prio = TxPriority::NORMAL;
    } else if (_cL > 0U) {
        slot = &_qLow[_hL];
        prio = TxPriority::LOW;
    } else {
        return false;
    }

    if (HAL_UART_Transmit_DMA(_huart,
                              reinterpret_cast<uint8_t*>(slot->data),
                              slot->len) == HAL_OK) {
        _txBusy = true;
        _txActivePriority = prio;
        return true;
    }

    _txBusy = false;
    _txActivePriority = TxPriority::NONE;
    return false;
}

void TerminalSerial::popActiveQueue() {
    switch (_txActivePriority) {
        case TxPriority::HIGH:
            if (_cH > 0U) {
                _hH = static_cast<uint8_t>((_hH + 1U) % TX_QUEUE_LEN);
                _cH--;
            }
            break;

        case TxPriority::NORMAL:
            if (_cN > 0U) {
                _hN = static_cast<uint8_t>((_hN + 1U) % TX_QUEUE_LEN);
                _cN--;
            }
            break;

        case TxPriority::LOW:
            if (_cL > 0U) {
                _hL = static_cast<uint8_t>((_hL + 1U) % TX_QUEUE_LEN);
                _cL--;
            }
            break;

        case TxPriority::NONE:
        default:
            break;
    }

    _txActivePriority = TxPriority::NONE;
}

bool TerminalSerial::enviarACK(const char* msg) {
    uint32_t p = enterCritical();

    const bool ok = enqueue(TxPriority::HIGH, msg);
    if (ok && !_txBusy) {
        iniciarTx();
    }

    exitCritical(p);
    return ok;
}

bool TerminalSerial::enviarNormal(const char* msg) {
    uint32_t p = enterCritical();

    const bool ok = enqueue(TxPriority::NORMAL, msg);
    if (ok && !_txBusy) {
        iniciarTx();
    }

    exitCritical(p);
    return ok;
}

bool TerminalSerial::enviarLog(const char* msg) {
    uint32_t p = enterCritical();

    const bool ok = enqueue(TxPriority::LOW, msg);
    if (ok && !_txBusy) {
        iniciarTx();
    }

    exitCritical(p);
    return ok;
}

bool TerminalSerial::enviarMensaje(const char* msg) {
    return enviarNormal(msg);
}

void TerminalSerial::onTxCompleteISR() {
    uint32_t p = enterCritical();

    popActiveQueue();
    _txBusy = false;
    iniciarTx();

    exitCritical(p);
}

void TerminalSerial::onTxErrorISR() {
    uint32_t p = enterCritical();

    _txBusy = false;
    _txActivePriority = TxPriority::NONE;
    iniciarTx();

    exitCritical(p);
}

void TerminalSerial::onRxErrorISR() {
    uint32_t p = enterCritical();

    _rxIndex = 0U;
    _rxBuildBuffer[0] = '\0';
    _rxOverflow = false;
    _rxQueueOverflow = false;

    _rxHead = 0U;
    _rxTail = 0U;
    _rxCount = 0U;
    memset(_rxQueue, 0, sizeof(_rxQueue));

    HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);

    exitCritical(p);
}

void TerminalSerial::interrupcionRecepcionByte() {
    if (_rxByte == '\r') {
        /* Ignorar CR para soportar terminales con \r\n. */
    }
    else if (_rxByte == '\n') {
        if (!_rxOverflow && (_rxIndex > 0U)) {
            _rxBuildBuffer[_rxIndex] = '\0';

            if (_rxCount < RX_QUEUE_LEN) {
                memcpy(_rxQueue[_rxTail], _rxBuildBuffer, RX_LINE_SIZE);
                _rxQueue[_rxTail][RX_LINE_SIZE - 1U] = '\0';
                _rxTail = static_cast<uint8_t>((_rxTail + 1U) % RX_QUEUE_LEN);
                _rxCount++;
            } else {
                _rxQueueOverflow = true;
            }
        }

        memset(_rxBuildBuffer, 0, sizeof(_rxBuildBuffer));
        _rxIndex = 0U;
        _rxOverflow = false;
    }
    else {
        if (_rxIndex < (RX_LINE_SIZE - 1U)) {
            _rxBuildBuffer[_rxIndex++] = static_cast<char>(_rxByte);
        } else {
            _rxOverflow = true;
        }
    }

    HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);
}

bool TerminalSerial::hayComandoPendiente() const {
    return (_rxCount > 0U);
}

const char* TerminalSerial::obtenerComandoPendiente() const {
    if (_rxCount == 0U) {
        return "";
    }

    return _rxQueue[_rxHead];
}

void TerminalSerial::limpiarComandoPendiente() {
    uint32_t p = enterCritical();

    if (_rxCount > 0U) {
        memset(_rxQueue[_rxHead], 0, RX_LINE_SIZE);
        _rxHead = static_cast<uint8_t>((_rxHead + 1U) % RX_QUEUE_LEN);
        _rxCount--;
    }

    exitCritical(p);
}
