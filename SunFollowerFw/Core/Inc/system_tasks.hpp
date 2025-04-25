#pragma once
#include <string>

namespace SystemTasks {
    // LED Steuerung
    std::string setLedState(const std::string& state);

    // Status & Geräteinfo
    std::string getStatus();
    std::string getID();

    // Measure Task Steuerung
    std::string startMeasureTask();
    std::string stopMeasureTask();

    // Blinking Task Steuerung
    std::string startBlinkingTask();
    std::string stopBlinkingTask();
}
