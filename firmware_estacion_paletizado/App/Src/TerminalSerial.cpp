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
      _rxBuffers{0},
	  _rxHead(0), _rxTail(0), _rxCount(0),
      _rxIndex(0),
      _rxOverflow(false),
      _qHigh{}, _qNormal{}, _qLow{},
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
 * ENCOLADO Y GESTION DE TRANSMISION (TX)
 * ================================================================
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

    // Si la cola esta llena no se puede encolar
    if (*c >= TX_QUEUE_LEN) return false;


    //Copiamos el mensaje al slot apuntado por tail.
    TxSlot& s = q[*t];
    memcpy(s.data, msg, len);
    // Un mensaje mas largo que el slot se recorta y pierde su "\r\n": la PC lo pegaria con el
    // mensaje siguiente y descartaria los dos (p. ej. un log largo seguido de un EVT:). Se fuerza
    // el fin de linea en el recorte para que cada slot sea siempre una linea completa.
    if ((len == TX_BUFFER_SIZE - 1U) && (msg[len] != '\0')) {
        s.data[len - 2U] = '\r';
        s.data[len - 1U] = '\n';
    }
    s.data[len] = '\0';
    s.len = static_cast<uint16_t>(len);

    //Avanzamos tail circularmente y aumentamos count.
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
    if (_txBusy) return false;

    TxSlot* slot = nullptr;
    TxPriority prio = TxPriority::NONE;

    // Seleccion de Prioridad
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


    // Si el DMA arranca correctamente: TX ocupada y Prioridad activa
    if (HAL_UART_Transmit_DMA(_huart,
                              reinterpret_cast<uint8_t*>(slot->data),
                              slot->len) == HAL_OK) {
        _txBusy = true;
        _txActivePriority = prio;
        return true;
    }

    //Si el DMA no pudo arrancar, no se consume nada.
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
 * RECEPCIÓN BYTE A BYTE (RX RING BUFFER)
 * ================================================================
 */
void TerminalSerial::onRxErrorISR() {
    uint32_t p = enterCritical();

    // Reseteamos el comando en construccion actual
    _rxIndex = 0U;
    _rxOverflow = false;
    _rxBuffers[_rxTail][0] = '\0';

    HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);

    exitCritical(p);
}

// Recepcion Byte a Byte
void TerminalSerial::interrupcionRecepcionByte() {
    /*
     * Control de flujo: Si el buffer circular esta lleno, descartamos
     * los bytes nuevos para proteger la memoria, hasta que el main loop
     * procese comandos mediante limpiarComandoPendiente().
     */
	if (_rxCount >= RX_QUEUE_LEN) {
		// Cola llena: el byte se pierde y la linea en curso queda incompleta. Hay que descartarla
		// entera; si no, su resto se pegaria al fragmento ya guardado (comando corrupto).
		if (_rxByte == '\n') {
			// Fin de la linea perdida: se descarta el fragmento y la proxima arranca limpia
			_rxIndex = 0U;
			_rxOverflow = false;
		} else if (_rxByte != '\r') {
			// Byte de datos perdido: descartar el resto de la linea hasta su '\n'
			_rxOverflow = true;
		}
		HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);
		return;
	}

	if (_rxByte == '\r') {
		// Ignoramos CR (\r) para limpiar formato nativo.
	} else if (_rxByte == '\n') {
		if (_rxOverflow) {
			// Descartamos esta línea malformada/muy larga, pero mantenemos estado FIFO.
			_rxIndex = 0U;
			_rxOverflow = false;
		} else {
			// Línea completada con éxito. Colocamos fin de string.
			_rxBuffers[_rxTail][_rxIndex] = '\0';

	        // Avanzamos el puntero de escritura de forma circular.
	        _rxTail = static_cast<uint8_t>((_rxTail + 1U) % RX_QUEUE_LEN);
	        _rxCount++;

	        // Preparamos índice para el siguiente comando
	        _rxIndex = 0U;
	    }
	} else {
		if (_rxIndex < (RX_STRING_MAX_LEN - 1U)) {
			_rxBuffers[_rxTail][_rxIndex++] = static_cast<char>(_rxByte);
		} else {
			_rxOverflow = true;
		}
	}
	// Rearme de la interrupción
	HAL_UART_Receive_IT(_huart, const_cast<uint8_t*>(&_rxByte), 1);
}

/*
 * ================================================================
 * CONSULTA Y LIMPIEZA DE COMANDOS RX
 * ================================================================
 */
bool TerminalSerial::hayComandoPendiente() const {
	return (_rxCount > 0U);
}

const char* TerminalSerial::obtenerComandoPendiente() const {
	if (_rxCount > 0U) {
		// Entregamos puntero a la cadena más antigua (Head)
	    return const_cast<const char*>(_rxBuffers[_rxHead]);
	}
	return nullptr;
}

void TerminalSerial::limpiarComandoPendiente() {
    // Al modificar _rxCount y _rxHead, competimos directamente con la ISR.
    // Es obligatorio entrar en sección crítica.
    uint32_t p = enterCritical();

    if (_rxCount > 0U) {
        // "Limpiamos" el comando simplemente avanzando el Head
        _rxHead = static_cast<uint8_t>((_rxHead + 1U) % RX_QUEUE_LEN);
        _rxCount--;
    }

    exitCritical(p);
}
