// scpi_parser.hpp
#pragma once

#include <string>
#include <vector>
#include "command.hpp"

class ScpiParser {
public:
    bool parse(const std::string& input, Command& outCmd);

private:
    void split(const std::string& input, char delimiter, std::vector<std::string>& tokens);
    std::string trim(const std::string& str);
    std::string toUpper(const std::string& str);
};