#pragma once
#include "command.hpp"
#include <string>
#include <vector>
#include <functional>

struct CommandHandler {
    std::vector<std::string> pathPattern;
    std::string subCmdPattern;
    bool isQuery;
    std::function<std::string(const Command&)> handlerFunc;
};

class CommandDispatcher {
public:
    CommandDispatcher();
    std::string dispatch(const Command& cmd);

private:
    std::vector<CommandHandler> handlers;

    // Handler
    std::string handleLedState(const Command& cmd);
    std::string handleStatusQuery(const Command& cmd);
    std::string handleIDQuery(const Command& cmd);
    std::string handleMeasureStart(const Command& cmd);
    std::string handleMeasureStop(const Command& cmd);
    std::string handleBlinkingStart(const Command& cmd);
    std::string handleBlinkingStop(const Command& cmd);
};
