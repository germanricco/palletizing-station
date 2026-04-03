#ifndef UARTCOMMAND_HPP
#define UARTCOMMAND_HPP

#include <stdint.h>

namespace Station {

/*
 * Comandos UART basados en códigos M.
 */
enum class UartCommand : uint8_t {
    None = 0,

    M100_ManualMode,         // IDLE -> MANUAL
    M101_AutoMode,           // IDLE -> AUTO_READY
    M110_StartRecording,     // MANUAL -> RECORDING
    M111_StopRecording,      // RECORDING -> MANUAL
    M112_StartPlayback,      // AUTO_READY -> AUTO_RUNNING
    M113_StopPausePlayback,  // AUTO_RUNNING -> AUTO_PAUSED o AUTO_PAUSED -> AUTO_READY
    M114_ResumePlayback,     // AUTO_PAUSED -> AUTO_RUNNING
    M99_GoIdle,              // estado operativo -> IDLE

    M00_EStop,               // parada de emergencia
    Status,                  // consulta de estado
    Unknown
};

const char* toString(UartCommand command);
UartCommand parseUartCommand(const char* text);

} // namespace Station

#endif /* UARTCOMMAND_HPP */
