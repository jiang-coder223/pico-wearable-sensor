# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(main C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# --- 核心設定：包含 main.c 和 MPU6050.c ---
add_executable(main 
    main.c 
    MPU6050.c 
    MAX30102.c
    MQTT.c
    SSD1306.c
)

pico_set_program_name(main "main")
pico_set_program_version(main "0.1")

# 輸出設定：關閉 UART，開啟 USB (Serial Monitor 才能看)
pico_enable_stdio_uart(main 0)
pico_enable_stdio_usb(main 1)

# --- 連結庫設定 ---
target_link_libraries(main 
    pico_stdlib 
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

# --- 包含目錄設定 ---
target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel/include
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM0
)

target_sources(main PRIVATE
    FreeRTOS-Kernel/tasks.c
    FreeRTOS-Kernel/queue.c
    FreeRTOS-Kernel/list.c
    FreeRTOS-Kernel/timers.c
    FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
    FreeRTOS-Kernel/portable/MemMang/heap_4.c
)
pico_add_extra_outputs(main)