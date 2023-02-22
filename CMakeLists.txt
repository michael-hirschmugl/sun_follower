cmake_minimum_required(VERSION 3.25.2)

project(sun-follower VERSION 0.0.0)

enable_language(CXX C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(STM32CUBEMX_GENERATED_FILES

        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_exti.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd_ex.c
        BSP/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_usb.c

        BSP/Inc/main.h
        BSP/Inc/stm32f0xx_hal_conf.h
        BSP/Inc/stm32f0xx_it.h

        BSP/Src/main.c
        BSP/Src/stm32f0xx_hal_msp.c
        BSP/Src/stm32f0xx_it.c
        BSP/Src/system_stm32f0xx.c

        BSP/startup_stm32f031x6.s)

set(EXECUTABLE ${PROJECT_NAME}.out)

add_executable(${EXECUTABLE} ${STM32CUBEMX_GENERATED_FILES}
        source/application.cpp
        source/application.hpp)

target_compile_definitions(${EXECUTABLE} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32F031x6
        )

target_include_directories(${EXECUTABLE} SYSTEM PRIVATE
        BSP/Inc
        BSP/Drivers/STM32F0xx_HAL_Driver/Inc
        BSP/Drivers/CMSIS/Device/ST/STM32F0xx/Include
        BSP/Drivers/CMSIS/Include)

target_include_directories(${EXECUTABLE} PRIVATE
        source/)

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m0
        -mthumb
        #-mfpu=fpv4-sp-d16
        #-mfloat-abi=hard

        -fdata-sections
        -ffunction-sections

        -fno-exceptions
        $<$<COMPILE_LANGUAGE:CXX>:@${CMAKE_SOURCE_DIR}/gcc-options-cxx.txt>

        -Wall

        $<$<CONFIG:Debug>:-Og>
        )


target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/BSP/STM32F031K6Tx_FLASH.ld
        -mcpu=cortex-m0
        -mthumb
        #-mfpu=fpv4-sp-d16
        #-mfloat-abi=hard
        -specs=nosys.specs
        -lc
        -lm
        -lnosys
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        )

add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE})

add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)

set_target_properties(${EXECUTABLE} PROPERTIES ADDITIONAL_CLEAN_FILES
        "${PROJECT_NAME}.bin;${PROJECT_NAME}.hex;${PROJECT_NAME}.map")
