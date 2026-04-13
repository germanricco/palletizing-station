#include "TerminalSerial.hpp"
#include <cstdio>
#include <cstring>

/*
 * ================================================================
 * CONSTRUCTOR
 * ================================================================
 *
 * Inicializa todos los índices, contadores y flags internos.
 */
TerminalSerial::TerminalSerial(UART_HandleTypeDef* huart)
    : _huart(huart),
      _rxByte(0),
      _rxBuffer{0},
      _rxIndex(0),
      _comandoListo(false),
      _rxOverflow(false),
      _qHigh{},
      _qNormal{},
      _qLow{},
      _hH(0), _tH(0), _cH(0),
      _hN(0), _tN(0), _cN(0),
      _hL(0), _tL(0), _cL(0),
      _txBusy(false),
      _txActivePriority(TxPriority::NONE) {
}

/*
 * ================================================================
 * INICIALIZACIÓN
 * ================================================================
 *
 * Inicia la recepción UART de a un byte por interrupción.
 */
void TerminalSerial::init() {
    HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);
}

/*
 * Hook de actualización.
 * Se deja por compatibilidad/extensión futura.
 */
void TerminalSerial::update() {
}

/*
 * ================================================================
 * SECCIONES CRÍTICAS
 * ================================================================
 *
 * Se utilizan para proteger colas y estados compartidos entre
 * ISR y contexto principal.
 */
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

/*
 * ================================================================
 * ENCOLADO DE MENSAJES
 * ================================================================
 *
 * Inserta un mensaje en la cola correspondiente a la prioridad.
 */
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

    /*
     * Si la cola está llena, no se puede encolar.
     */
    if (*c >= TX_QUEUE_LEN) {
        return false;
    }

    /*
     * Copiamos el mensaje al slot apuntado por tail.
     */
    TxSlot& s = q[*t];
    memcpy(s.data, msg, len);
    s.data[len] = '\0';
    s.len = static_cast<uint16_t>(len);

    /*
     * Avanzamos tail circularmente y aumentamos count.
     */
    *t = static_cast<uint8_t>((*t + 1U) % TX_QUEUE_LEN);
    (*c)++;

    return true;
}

/*
 * ================================================================
 * ARRANQUE DE TRANSMISIÓN DMA
 * ================================================================
 *
 * Si no hay TX activa, toma el mensaje de mayor prioridad disponible
 * e intenta iniciar el DMA.
 */
bool TerminalSerial::iniciarTx() {
    if (_txBusy) {
        return false;
    }

    TxSlot* slot = nullptr;
    TxPriority prio = TxPriority::NONE;

    /*
     * Selección por prioridad:
     * HIGH > NORMAL > LOW
     */
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

    /*
     * Si el DMA arranca correctamente, se marca:
     * - TX ocupada
     * - prioridad activa
     */
    if (HAL_UART_Transmit_DMA(_huart,
                              reinterpret_cast<uint8_t*>(slot->data),
                              slot->len) == HAL_OK) {
        _txBusy = true;
        _txActivePriority = prio;
        return true;
    }

    /*
     * Si el DMA no pudo arrancar, no se consume nada.
     */
    _txBusy = false;
    _txActivePriority = TxPriority::NONE;
    return false;
}

/*
 * ================================================================
 * POP DE LA COLA ACTIVA
 * ================================================================
 *
 * Descarta exactamente el elemento que terminó de transmitirse.
 * Esto evita el bug de descontar la cola equivocada.
 */
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

/*
 * ================================================================
 * API DE ENVÍO
 * ================================================================
 *
 * Cada función:
 * - entra en sección crítica
 * - encola el mensaje
 * - si no hay TX activa, intenta arrancarla
 */
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

/*
 * ================================================================
 * CALLBACKS DE TX
 * ================================================================
 */

/*
 * Callback de fin de transmisión DMA.
 *
 * Flujo:
 * 1) Se descuenta el elemento realmente transmitido.
 * 2) Se marca TX libre.
 * 3) Se intenta iniciar la siguiente TX pendiente.
 */
void TerminalSerial::onTxCompleteISR() {
    uint32_t p = enterCritical();

    popActiveQueue();
    _txBusy = false;
    iniciarTx();

    exitCritical(p);
}

/*
 * Callback de error en TX.
 *
 * Estrategia:
 * - liberar estado busy
 * - no consumir cola
 * - limpiar prioridad activa
 * - reintentar transmisión
 */
void TerminalSerial::onTxErrorISR() {
    uint32_t p = enterCritical();

    _txBusy = false;
    _txActivePriority = TxPriority::NONE;
    iniciarTx();

    exitCritical(p);
}

/*
 * ================================================================
 * CALLBACK DE ERROR EN RX
 * ================================================================
 *
 * Recupera el estado de recepción ante errores UART.
 */
void TerminalSerial::onRxErrorISR() {
    uint32_t p = enterCritical();

    _rxIndex = 0U;
    _rxBuffer[0] = '\0';
    _comandoListo = false;
    _rxOverflow = false;

    HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);

    exitCritical(p);
}

/*
 * ================================================================
 * RECEPCIÓN BYTE A BYTE
 * ================================================================
 *
 * Política:
 * - '\r' se ignora
 * - '\n' cierra la línea
 * - si ya hay un comando pendiente, se descartan nuevos bytes
 *   para no sobrescribir la línea todavía no procesada
 * - si el buffer se llena, se descarta el resto de la línea actual
 */
void TerminalSerial::interrupcionRecepcionByte() {
    /*
     * Si ya hay una línea completa pendiente de ser procesada,
     * descartamos bytes nuevos hasta que el loop principal la consuma.
     */
    if (_comandoListo) {
        HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);
        return;
    }

    if (_rxByte == '\r') {
        /*
         * Se ignora CR para soportar terminales con "\r\n".
         */
    } else if (_rxByte == '\n') {
        /*
         * Si la línea sufrió overflow, se descarta completa.
         */
        if (_rxOverflow) {
            _rxBuffer[0] = '\0';
            _rxIndex = 0U;
            _rxOverflow = false;
            _comandoListo = false;
        } else {
            _rxBuffer[_rxIndex] = '\0';
            _comandoListo = true;
        }
    } else {
        if (_rxIndex < (RX_BUFFER_SIZE - 1U)) {
            _rxBuffer[_rxIndex++] = static_cast<char>(_rxByte);
        } else {
            /*
             * No se escribe fuera del buffer.
             * Se marca overflow y se descarta el resto de la línea.
             */
            _rxOverflow = true;
        }
    }

    /*
     * Rearmado de la recepción del próximo byte.
     */
    HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);
}

/*
 * ================================================================
 * CONSULTA Y LIMPIEZA DE COMANDOS RX
 * ================================================================
 */
bool TerminalSerial::hayComandoPendiente() const {
    return _comandoListo;
}

const char* TerminalSerial::obtenerComandoPendiente() const {
    return _rxBuffer;
}

void TerminalSerial::limpiarComandoPendiente() {
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
    _rxIndex = 0U;
    _comandoListo = false;
    _rxOverflow = false;
}
