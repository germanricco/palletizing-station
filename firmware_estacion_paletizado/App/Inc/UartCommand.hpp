#ifndef UARTCOMMAND_HPP
#define UARTCOMMAND_HPP

#include <stdint.h>

namespace Station {

enum class UartCommand : uint8_t {
    None = 0,
    Manual,
    Auto,
    Stop,
    Pause,
    Resume,
    Reset,
    EStop,
    Home,
    Status,
    Unknown
};

const char* toString(UartCommand command);
UartCommand parseUartCommand(const char* text);

} // namespace Station

#endif /* UARTCOMMAND_HPP */
