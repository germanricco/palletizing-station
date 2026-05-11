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
 * 1) Recibir comandos UART mediante interrupción y guardarlos en
 *    una cola FIFO circular para soportar transmisiones en ráfaga (Burst).
 * 2) Enviar mensajes por UART usando DMA sin bloquear el procesador.
 * 3) Gestionar colas de transmisión con prioridad (HIGH, NORMAL, LOW).
 *
 * Mejoras Arquitectónicas:
 * - RX Queue: Se reemplaza el buffer único por una matriz circular de 64
 *   posiciones. Esto soluciona la pérdida de datos cuando Python envía
 *   chunks de 32 waypoints a alta velocidad.
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
     */
    struct TxSlot {
        char data[96];
        uint16_t len;
    };

public:
    explicit TerminalSerial(UART_HandleTypeDef* huart);


    void init();
    void update();

    // API de Transmision
    bool enviarACK(const char* msg);
    bool enviarNormal(const char* msg);
    bool enviarLog(const char* msg);
    bool enviarMensaje(const char* msg);

    // Callbacks ISR / HAL
    void onTxCompleteISR();
    void onTxErrorISR();
    void onRxErrorISR();
    void interrupcionRecepcionByte();

    // API de Recepcion
    bool hayComandoPendiente() const;
    const char* obtenerComandoPendiente() const;
    void limpiarComandoPendiente();

private:
    /*
     * ------------------------------------------------
     * Parámetros de dimensionamiento
     * ------------------------------------------------
     */
    static constexpr uint8_t  RX_QUEUE_LEN 	 	= 32;	// Capacidad de comandos encolados (Para chunks de 32)
    static constexpr uint16_t RX_STRING_MAX_LEN = 32;	// Largo máximo de CADA comando (Ej: G1:1.0,2.0,3.0,1)
    static constexpr uint16_t TX_BUFFER_SIZE 	= 96;	// Largo máximo de TX. Emparejado con el TxSlot
    static constexpr uint8_t  TX_QUEUE_LEN   	= 16;	// Profundidad de colas TX

    UART_HandleTypeDef* _huart;

    /*
     * ------------------------------------------------
     * RX - Cola Circular de Comandos (Ring Buffer)
     * ------------------------------------------------
     */
    volatile uint8_t _rxByte;
    char _rxBuffers[RX_QUEUE_LEN][RX_STRING_MAX_LEN];

    volatile uint8_t _rxHead;	// Puntero de lectura (Main Loop)
    volatile uint8_t _rxTail;	// Puntero de escritura (ISR UART)
    volatile uint8_t _rxCount;	// Cantidad de líneas/comandos completos listos

    volatile uint16_t _rxIndex;	// Índice del byte en la línea actual (_rxTail)
    volatile bool _rxOverflow;	// Bandera de seguridad por línea excesivamente larga

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

    // Estado TX activa
    volatile bool _txBusy;
    volatile TxPriority _txActivePriority;

private:
    // Utilidades Internas
    bool enqueue(TxPriority prio, const char* msg);
    bool iniciarTx();
    void popActiveQueue();

    uint32_t enterCritical();
    void exitCritical(uint32_t primask);
};

#endif	// TERMINALSERIAL_HPP
