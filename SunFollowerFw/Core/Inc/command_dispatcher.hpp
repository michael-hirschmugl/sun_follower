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

    // Deine Handler
    std::string handleLedState(const Command& cmd);
    std::string handleStatusQuery(const Command& cmd);
};
