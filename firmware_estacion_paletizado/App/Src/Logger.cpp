/**
 * @file Logger.cpp
 * @brief Implementación de las rutinas de formato y despacho de la clase Logger.
 */

#include "Logger.hpp"
#include <cstdio>
#include <cstring>

// Inicialización de variables estáticas en el segmento de datos
TerminalSerial* Logger::_terminal = nullptr;
LogLevel Logger::_nivelFiltroActual = LogLevel::Info;
char Logger::_bufferFormato[Logger::LOG_BUFFER_SIZE] = {0};

void Logger::init(TerminalSerial* terminal, LogLevel nivel) {
    _terminal = terminal;
    _nivelFiltroActual = nivel;
}

void Logger::setNivel(LogLevel nivel) {
    _nivelFiltroActual = nivel;
}

void Logger::formatearYEnviar(LogLevel nivel, const char* formato, va_list args) {
    // Barreras de seguridad operativa
    if (_terminal == nullptr || nivel > _nivelFiltroActual) {
        return;
    }

    const char* prefijoNivel = "";

    // Actualización de los identificadores para coincidir con la nueva declaración
    switch (nivel) {
            case LogLevel::Error: prefijoNivel = "[ERROR]"; break;
            case LogLevel::Info:  prefijoNivel = "[INFO] "; break;
            case LogLevel::Debug: prefijoNivel = "[DEBUG]"; break;
        }

    // 1. Obtencion del tiempo de ejecucion por hardware
    uint32_t marcaTiempoMs = HAL_GetTick();

    // 2. Construccion del prefijo compuesto
    // Formato estructurado resultante: "[00001050] [INFO] "
    int longitudPrefijo = snprintf(_bufferFormato, LOG_BUFFER_SIZE, "[%08lu] %s ", marcaTiempoMs, prefijoNivel);

    // Validación estricta de límites para la prevención de desbordamientos de pila/BSS
    if (longitudPrefijo > 0 && longitudPrefijo < static_cast<int>(LOG_BUFFER_SIZE)) {
    	// 3. Formateo variádico de la carga útil empleando aritmética de punteros
        // Se restan 3 bytes para aislar la memoria requerida por el terminador UART "\r\n\0"
        vsnprintf(_bufferFormato + longitudPrefijo,
        		LOG_BUFFER_SIZE - longitudPrefijo - 3,
				formato, args);

        // 4. Terminación estricta exigida por los emuladores de terminal serial estándar
        strncat(_bufferFormato, "\r\n", LOG_BUFFER_SIZE - strlen(_bufferFormato) - 1);

        // 5. Invocación de la transferencia Direct Memory Access (DMA)
        _terminal->enviarMensaje(_bufferFormato);
    }
}

void Logger::logERROR(const char* formato, ...) {
    va_list args;
    va_start(args, formato);
    formatearYEnviar(LogLevel::Error, formato, args);
    va_end(args);
}

void Logger::logINFO(const char* formato, ...) {
    va_list args;
    va_start(args, formato);
    formatearYEnviar(LogLevel::Info, formato, args);
    va_end(args);
}

void Logger::logDEBUG(const char* formato, ...) {
    va_list args;
    va_start(args, formato);
    formatearYEnviar(LogLevel::Debug, formato, args);
    va_end(args);
}
