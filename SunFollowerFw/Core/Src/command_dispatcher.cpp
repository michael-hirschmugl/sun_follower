#include "command_dispatcher.hpp"
#include "gpio.h"

CommandDispatcher::CommandDispatcher() {
    handlers = {
        { {"LED"}, "STATE", false, [this](const Command& cmd) { return handleLedState(cmd); }},
        { {}, "STATUS", true, [this](const Command& cmd) { return handleStatusQuery(cmd); }}
    };
}

std::string CommandDispatcher::dispatch(const Command& cmd) {
    for (const auto& handler : handlers) {
        if (cmd.path == handler.pathPattern &&
            cmd.subCmd == handler.subCmdPattern &&
            cmd.isQuery == handler.isQuery) 
        {
            return handler.handlerFunc(cmd);
        }
    }
    return "ERR: Unknown Command\r\n";
}

std::string CommandDispatcher::handleLedState(const Command& cmd) {
    (void)cmd;
    if (cmd.params.empty()) return "ERR: Missing Parameter\r\n";

    if (cmd.params[0] == "ON") {
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
        return "LED ON\r\n";
    } else if (cmd.params[0] == "OFF") {
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
        return "LED OFF\r\n";
    }
    return "ERR: Invalid Param\r\n";
}

std::string CommandDispatcher::handleStatusQuery(const Command& cmd) {
    (void)cmd;
    return "STATUS: OK\r\n";
}
