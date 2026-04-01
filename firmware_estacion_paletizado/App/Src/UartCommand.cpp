#include "UartCommand.hpp"
#include <cstring>
#include <cctype>

namespace Station {

static void normalizeToUpperNoSpaces(const char* src, char* dst, uint16_t dstSize) {
    if ((src == nullptr) || (dst == nullptr) || (dstSize == 0U)) {
        return;
    }

    uint16_t j = 0U;
    for (uint16_t i = 0U; src[i] != '\0' && j < (dstSize - 1U); ++i) {
        const unsigned char c = static_cast<unsigned char>(src[i]);
        if ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n')) {
            continue;
        }
        dst[j++] = static_cast<char>(std::toupper(c));
    }
    dst[j] = '\0';
}

const char* toString(UartCommand command) {
    switch (command) {
        case UartCommand::None:    return "None";
        case UartCommand::Manual:  return "Manual";
        case UartCommand::Auto:    return "Auto";
        case UartCommand::Stop:    return "Stop";
        case UartCommand::Pause:   return "Pause";
        case UartCommand::Resume:  return "Resume";
        case UartCommand::Reset:   return "Reset";
        case UartCommand::EStop:   return "EStop";
        case UartCommand::Home:    return "Home";
        case UartCommand::Status:  return "Status";
        default:                   return "Unknown";
    }
}

UartCommand parseUartCommand(const char* text) {
    char cmd[32] = {0};
    normalizeToUpperNoSpaces(text, cmd, sizeof(cmd));

    if (std::strcmp(cmd, "MANUAL") == 0) {
        return UartCommand::Manual;
    }
    if (std::strcmp(cmd, "AUTO") == 0) {
        return UartCommand::Auto;
    }
    if (std::strcmp(cmd, "STOP") == 0) {
        return UartCommand::Stop;
    }
    if (std::strcmp(cmd, "PAUSE") == 0) {
        return UartCommand::Pause;
    }
    if (std::strcmp(cmd, "RESUME") == 0) {
        return UartCommand::Resume;
    }
    if (std::strcmp(cmd, "RESET") == 0) {
        return UartCommand::Reset;
    }
    if (std::strcmp(cmd, "ESTOP") == 0) {
        return UartCommand::EStop;
    }
    if (std::strcmp(cmd, "HOME") == 0) {
        return UartCommand::Home;
    }
    if (std::strcmp(cmd, "STATUS") == 0) {
        return UartCommand::Status;
    }

    if (cmd[0] == '\0') {
        return UartCommand::None;
    }

    return UartCommand::Unknown;
}

} // namespace Station

