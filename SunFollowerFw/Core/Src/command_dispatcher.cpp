#include "command_dispatcher.hpp"
#include "system_tasks.hpp"

CommandDispatcher::CommandDispatcher() {
    handlers = {
        { {"LED"}, "STATE", false, [](const Command& cmd) { 
            return cmd.params.empty() ? "ERR: Missing Parameter\r\n" : SystemTasks::setLedState(cmd.params[0]); 
        }},
        { {}, "STATUS", true, [](const Command&) { return SystemTasks::getStatus(); }},
        { {}, "*IDN", true, [](const Command&) { return SystemTasks::getID(); }},
        { {"MEASURE"}, "START", false, [](const Command&) { return SystemTasks::startMeasureTask(); }},
        { {"MEASURE"}, "STOP",  false, [](const Command&) { return SystemTasks::stopMeasureTask(); }},
        { {"BLINK"}, "START", false, [](const Command&) { return SystemTasks::startBlinkingTask(); }},
        { {"BLINK"}, "STOP",  false, [](const Command&) { return SystemTasks::stopBlinkingTask(); }}
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
