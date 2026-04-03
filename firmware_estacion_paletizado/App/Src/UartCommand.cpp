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
        case UartCommand::None:                   return "None";
        case UartCommand::M100_ManualMode:        return "M100";
        case UartCommand::M101_AutoMode:          return "M101";
        case UartCommand::M110_StartRecording:    return "M110";
        case UartCommand::M111_StopRecording:     return "M111";
        case UartCommand::M112_StartPlayback:     return "M112";
        case UartCommand::M113_StopPausePlayback: return "M113";
        case UartCommand::M114_ResumePlayback:    return "M114";
        case UartCommand::M99_GoIdle:             return "M99";
        case UartCommand::M00_EStop:              return "M00";
        case UartCommand::Status:                 return "STATUS";
        default:                                  return "Unknown";
    }
}

UartCommand parseUartCommand(const char* text) {
    char cmd[32] = {0};
    normalizeToUpperNoSpaces(text, cmd, sizeof(cmd));

    if (cmd[0] == '\0') {
        return UartCommand::None;
    }

    if (std::strcmp(cmd, "M100") == 0) {
        return UartCommand::M100_ManualMode;
    }
    if (std::strcmp(cmd, "M101") == 0) {
        return UartCommand::M101_AutoMode;
    }
    if (std::strcmp(cmd, "M110") == 0) {
        return UartCommand::M110_StartRecording;
    }
    if (std::strcmp(cmd, "M111") == 0) {
        return UartCommand::M111_StopRecording;
    }
    if (std::strcmp(cmd, "M112") == 0) {
        return UartCommand::M112_StartPlayback;
    }
    if (std::strcmp(cmd, "M113") == 0) {
        return UartCommand::M113_StopPausePlayback;
    }
    if (std::strcmp(cmd, "M114") == 0) {
        return UartCommand::M114_ResumePlayback;
    }
    if (std::strcmp(cmd, "M99") == 0) {
        return UartCommand::M99_GoIdle;
    }
    if (std::strcmp(cmd, "M00") == 0) {
        return UartCommand::M00_EStop;
    }
    if (std::strcmp(cmd, "STATUS") == 0) {
        return UartCommand::Status;
    }

    return UartCommand::Unknown;
}

} // namespace Station
