#include "command.hpp"
#include <sstream>

std::string Command::toString() const {
    std::stringstream ss;
    ss << "> Parsed Command: :";
    for (const auto& p : path) {
        ss << p << ":";
    }
    ss << subCmd;
    if (isQuery) ss << "?";
    ss << "\r\n";

    if (!params.empty()) {
        ss << "> Params: ";
        for (size_t i = 0; i < params.size(); ++i) {
            ss << params[i];
            if (i != params.size() - 1) ss << ",";
        }
        ss << "\r\n";
    }

    //ss << "\r\n";   // Leerzeile
    return ss.str();
}


