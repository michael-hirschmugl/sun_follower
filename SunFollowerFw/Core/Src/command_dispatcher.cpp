#include "command_dispatcher.hpp"
#include "gpio.h"
#include "cmsis_os.h"
#include "usart.h"

// Forward Declaration für den Task und das Handle
extern "C" void MeasureTask(void* pvParameters);
extern "C" void BlinkingTask(void* pvParameters);
extern TaskHandle_t measureTaskHandle;
extern TaskHandle_t blinkingTaskHandle;

CommandDispatcher::CommandDispatcher() {
    handlers = {
        { {"LED"}, "STATE", false, [this](const Command& cmd) { return handleLedState(cmd); }},
        { {}, "STATUS", true, [this](const Command& cmd) { return handleStatusQuery(cmd); }},
        { {}, "*IDN", true, [this](const Command& cmd) { return handleIDQuery(cmd); }},
        { {"MEASURE"}, "START", false, [this](const Command& cmd) { return handleMeasureStart(cmd); }},
        { {"MEASURE"}, "STOP",  false, [this](const Command& cmd) { return handleMeasureStop(cmd); }},
        { {"BLINK"}, "START", false, [this](const Command& cmd) { return handleBlinkingStart(cmd); }},
        { {"BLINK"}, "STOP",  false, [this](const Command& cmd) { return handleBlinkingStop(cmd); }}
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

std::string CommandDispatcher::handleIDQuery(const Command& cmd) {
    (void)cmd;
    return "SunFollower v0.0.0\r\n";
}

std::string CommandDispatcher::handleMeasureStart(const Command& cmd) {
    (void)cmd;
    if (measureTaskHandle != nullptr) {
        return "MeasureTask already running\r\n";
    }
    xTaskCreate(MeasureTask, "MeasureTask", 256, nullptr, 2, &measureTaskHandle);
    return "MeasureTask started\r\n";
}

std::string CommandDispatcher::handleMeasureStop(const Command& cmd) {
    (void)cmd;
    if (measureTaskHandle == nullptr) {
        return "MeasureTask not running\r\n";
    }
    vTaskDelete(measureTaskHandle);
    measureTaskHandle = nullptr;
    return "MeasureTask stopped\r\n";
}

std::string CommandDispatcher::handleBlinkingStart(const Command& cmd) {
    (void)cmd;
    if (blinkingTaskHandle != nullptr) {
        return "BlinkingTask already running\r\n";
    }
    xTaskCreate(BlinkingTask, "BlinkingTask", 256, nullptr, 2, &blinkingTaskHandle);
    return "BlinkingTask started\r\n";
}

std::string CommandDispatcher::handleBlinkingStop(const Command& cmd) {
    (void)cmd;
    if (blinkingTaskHandle == nullptr) {
        return "BlinkingTask not running\r\n";
    }
    vTaskDelete(blinkingTaskHandle);
    blinkingTaskHandle = nullptr;
    return "BlinkingTask stopped\r\n";
}