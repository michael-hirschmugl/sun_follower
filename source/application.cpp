#include "application.hpp"

#include "main.h"

extern "C" {

void setup() {
    
}

void loop(UART_HandleTypeDef huart1) {

    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    //HAL_Delay(800U);
    //sprintf(msg, "Working\r\n");
    //HAL_UART_Transmit(&huart1, (unsigned char *)("Working\r\n"), sizeof("Working\r\n"), 1000);

}
}