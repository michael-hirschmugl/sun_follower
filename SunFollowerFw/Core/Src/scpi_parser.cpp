// scpi_parser.cpp
#include "scpi_parser.hpp"
#include <sstream>
#include <algorithm>

bool ScpiParser::parse(const std::string& input, Command& outCmd) {
    std::string cleaned = trim(input);
    if (cleaned.empty() || cleaned[0] != ':') return false;

    size_t spacePos = cleaned.find(' ');
    std::string cmdPart = cleaned.substr(0, spacePos);
    std::string paramPart = (spacePos != std::string::npos) ? cleaned.substr(spacePos + 1) : "";

    std::vector<std::string> tokens;
    split(cmdPart, ':', tokens);

    if (tokens.size() < 2) return false; // Muss mindestens einen Befehl enthalten

    outCmd.path.clear();
    for (size_t i = 1; i < tokens.size() - 1; ++i) {
        outCmd.path.push_back(toUpper(tokens[i]));
    }

    outCmd.subCmd = tokens.back();
    if (!outCmd.subCmd.empty() && outCmd.subCmd.back() == '?') {
        outCmd.isQuery = true;
        outCmd.subCmd.pop_back();
    } else {
        outCmd.isQuery = false;
    }
    outCmd.subCmd = toUpper(outCmd.subCmd);

    outCmd.params.clear();
    if (!paramPart.empty()) {
        split(paramPart, ',', outCmd.params);
    }

    return true;
}

void ScpiParser::split(const std::string& input, char delimiter, std::vector<std::string>& tokens) {
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        if (!item.empty()) tokens.push_back(item);
    }
}

std::string ScpiParser::trim(const std::string& str) {
    const char* whitespace = " \t\n\r";
    size_t start = str.find_first_not_of(whitespace);
    size_t end = str.find_last_not_of(whitespace);
    return (start == std::string::npos) ? "" : str.substr(start, end - start + 1);
}

std::string ScpiParser::toUpper(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
}
