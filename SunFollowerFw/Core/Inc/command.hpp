// command.hpp
#pragma once

#include <string>
#include <vector>

struct Command {
    std::vector<std::string> path;
    std::string subCmd;
    std::vector<std::string> params;
    bool isQuery = false;

    std::string toString() const;  // NEU
};
