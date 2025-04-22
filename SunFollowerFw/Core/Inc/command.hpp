// command.hpp
#pragma once

#include <string>
#include <vector>

struct Command {
    std::vector<std::string> path;   // z.B. {"LED", "STATE"}
    std::string subCmd;              // z.B. "ON" oder "?"
    std::vector<std::string> params; // z.B. {"100", "200"}
    bool isQuery = false;            // NEU: true wenn Befehl eine Abfrage ist
};