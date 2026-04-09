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
 * Clase encargada de:
 * 1) Recibir comandos UART byte a byte por interrupción.
 * 2) Armar líneas completas terminadas en '\n'.
 * 3) Enviar mensajes por UART usando DMA.
 * 4) Gestionar 3 colas de transmisión con prioridad:
 *    - HIGH   : ACK / respuestas críticas del protocolo
 *    - NORMAL : telemetría / eventos normales
 *    - LOW    : logs de depuración
 *
 * Además, esta versión corrige dos problemas importantes:
 * - Se guarda explícitamente qué cola quedó activa cuando arrancó
 *   una TX DMA, para luego descontar exactamente esa cola.
 * - Se protege el buffer RX para que una línea completa pendiente
 *   no sea sobrescrita por bytes nuevos antes de ser procesada.
 * ================================================================
 */
class TerminalSerial {
public:
    /*
     * ------------------------------------------------
     * Prioridades de transmisión
     * ------------------------------------------------
     */
    enum class TxPriority : uint8_t {
        HIGH = 0,
        NORMAL,
        LOW,
        NONE
    };

    /*
     * ------------------------------------------------
     * Slot de cola TX
     * ------------------------------------------------
     *
     * Almacena un mensaje ya copiado y su longitud efectiva.
     */
    struct TxSlot {
        char data[128];
        uint16_t len;
    };

public:
    /*
     * Constructor.
     * Recibe el handle UART que utilizará la terminal.
     */
    explicit TerminalSerial(UART_HandleTypeDef* huart);

    /*
     * Inicializa la recepción UART en modo interrupción byte a byte.
     */
    void init();

    /*
     * Hook de actualización.
     * Se deja por compatibilidad y extensión futura.
     */
    void update();

    /*
     * Envíos con distintas prioridades.
     */
    bool enviarACK(const char* msg);
    bool enviarNormal(const char* msg);
    bool enviarLog(const char* msg);

    /*
     * Alias histórico para compatibilidad.
     */
    bool enviarMensaje(const char* msg);

    /*
     * ------------------------------------------------
     * Callbacks ISR / HAL
     * ------------------------------------------------
     */
    void onTxCompleteISR();
    void onTxErrorISR();
    void onRxErrorISR();
    void interrupcionRecepcionByte();

    /*
     * ------------------------------------------------
     * Gestión de comandos RX
     * ------------------------------------------------
     */
    bool hayComandoPendiente() const;
    const char* obtenerComandoPendiente() const;
    void limpiarComandoPendiente();

private:
    /*
     * ------------------------------------------------
     * Parámetros internos
     * ------------------------------------------------
     */
    static constexpr uint16_t RX_BUFFER_SIZE = 128;
    static constexpr uint16_t TX_BUFFER_SIZE = 128;
    static constexpr uint8_t  TX_QUEUE_LEN   = 8;

    UART_HandleTypeDef* _huart;

    /*
     * ------------------------------------------------
     * RX
     * ------------------------------------------------
     */
    volatile uint8_t _rxByte;
    char _rxBuffer[RX_BUFFER_SIZE];
    volatile uint16_t _rxIndex;
    volatile bool _comandoListo;
    volatile bool _rxOverflow;

    /*
     * ------------------------------------------------
     * TX - Colas por prioridad
     * ------------------------------------------------
     */
    TxSlot _qHigh[TX_QUEUE_LEN];
    TxSlot _qNormal[TX_QUEUE_LEN];
    TxSlot _qLow[TX_QUEUE_LEN];

    /*
     * h = head  -> posición de salida
     * t = tail  -> posición de inserción
     * c = count -> cantidad de elementos
     */
    volatile uint8_t _hH, _tH, _cH;
    volatile uint8_t _hN, _tN, _cN;
    volatile uint8_t _hL, _tL, _cL;

    /*
     * Estado de TX activa.
     */
    volatile bool _txBusy;
    volatile TxPriority _txActivePriority;

private:
    /*
     * ------------------------------------------------
     * Utilidades internas
     * ------------------------------------------------
     */
    bool enqueue(TxPriority prio, const char* msg);
    bool iniciarTx();
    void popActiveQueue();

    uint32_t enterCritical();
    void exitCritical(uint32_t primask);
};

#endif
