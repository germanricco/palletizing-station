/**
 * @file Logger.hpp
 * @brief Implementación de un sistema de registro de eventos (Logging) no bloqueante.
 * @details Utiliza memoria estática y rutinas de formato variádico para evitar la fragmentación
 * del heap. Diseñado para acoplarse con la transmisión UART por DMA.
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <stdint.h>
#include <cstdarg>
#include "TerminalSerial.hpp"
#include "stm32f1xx_hal.h"		// Dependencia para el acceso al hardware SysTick
/**
 * @enum LogLevel
 * @brief Niveles jerárquicos de severidad para el filtrado de mensajes.
 * @note Se utiliza PascalCase para evitar colisiones con la macro del preprocesador.
 */
enum class LogLevel : uint8_t {
    Error = 0,
    Info = 1,
    Debug = 2
};

class Logger {
private:
    static TerminalSerial* _terminal;       /**< Puntero al despachador serial de hardware */
    static LogLevel _nivelFiltroActual;     /**< Umbral de severidad operativa */

    static constexpr size_t LOG_BUFFER_SIZE = 128;
    static char _bufferFormato[LOG_BUFFER_SIZE]; /**< Buffer estático reservado en memoria BSS */

    /**
     * @brief Motor central de formateo y despacho de cadenas.
     * @param nivel Nivel de severidad del mensaje entrante.
     * @param formato Cadena de formato estándar de C.
     * @param args Lista de argumentos variádicos resuelta.
     */
    static void formatearYEnviar(LogLevel nivel, const char* formato, va_list args);

public:
    /**
     * @brief Inyecta la dependencia del hardware de transmisión.
     * @param terminal Instancia configurada de la clase TerminalSerial.
     * @param nivel Nivel de depuración inicial (Por defecto: INFO).
     */
    static void init(TerminalSerial* terminal, LogLevel nivel = LogLevel::Info);

    /**
     * @brief Modifica el umbral de filtrado en tiempo de ejecución.
     * @param nivel Nuevo límite de severidad admisible.
     */
    static void setNivel(LogLevel nivel);

    /**
     * @brief Registra un error crítico en la ejecución del sistema.
     * @param formato Cadena literal con especificadores de formato (ej. "%d", "%f").
     * @param ... Argumentos variables asociados a los especificadores.
     * @note No debe ser invocado dentro de Rutinas de Servicio de Interrupción (ISR).
     */
    static void logERROR(const char* formato, ...);

    /**
     * @brief Registra información nominal sobre el estado del programa.
     * @param formato Cadena literal con especificadores de formato.
     */
    static void logINFO(const char* formato, ...);

    /**
     * @brief Registra datos de diagnóstico detallados (ej. valores de telemetría).
     * @param formato Cadena literal con especificadores de formato.
     */
    static void logDEBUG(const char* formato, ...);
};

#endif /* LOGGER_HPP */
