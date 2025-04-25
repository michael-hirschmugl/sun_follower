#include "system_tasks.hpp"
#include "gpio.h"
#include "cmsis_os.h"
#include "usart.h"
#include <cstring>

// Interne Task Handles
static TaskHandle_t measureTaskHandle = nullptr;
static TaskHandle_t blinkingTaskHandle = nullptr;

namespace SystemTasks {

// ---------- LED Funktion ----------
std::string setLedState(const std::string& state) {
    if (state == "ON") {
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
        return "LED ON\r\n";
    } else if (state == "OFF") {
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
        return "LED OFF\r\n";
    }
    return "ERR: Invalid Param\r\n";
}

// ---------- Status & ID ----------
std::string getStatus() {
    return "STATUS: OK\r\n";
}

std::string getID() {
    return "SunFollower v0.0.0\r\n";
}

// ---------- Measure Task ----------
static void MeasureTask(void* pv) {
    (void)pv;
    while (1) {
        const char* msg = "Measuring... Value = 42\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

std::string startMeasureTask() {
    if (measureTaskHandle != nullptr) return "MeasureTask already running\r\n";
    xTaskCreate(MeasureTask, "MeasureTask", 256, nullptr, 2, &measureTaskHandle);
    return "MeasureTask started\r\n";
}

std::string stopMeasureTask() {
    if (measureTaskHandle == nullptr) return "MeasureTask not running\r\n";
    vTaskDelete(measureTaskHandle);
    measureTaskHandle = nullptr;
    return "MeasureTask stopped\r\n";
}

// ---------- Blinking Task ----------
static void BlinkingTask(void* pv) {
    (void)pv;
    while (1) {
        const char* msg = "Blinking...\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

std::string startBlinkingTask() {
    if (blinkingTaskHandle != nullptr) return "BlinkingTask already running\r\n";
    xTaskCreate(BlinkingTask, "BlinkingTask", 256, nullptr, 2, &blinkingTaskHandle);
    return "BlinkingTask started\r\n";
}

std::string stopBlinkingTask() {
    if (blinkingTaskHandle == nullptr) return "BlinkingTask not running\r\n";
    vTaskDelete(blinkingTaskHandle);
    blinkingTaskHandle = nullptr;
    return "BlinkingTask stopped\r\n";
}

}  // namespace SystemTasks
