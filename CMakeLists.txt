# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

set(FREERTOS_KERNEL_PATH "C:/Users/Miller/Desktop/Univasf/Semestre III/Embarca/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(EmbarcaTech_EstacaoMetereo_BMP-AHT C CXX ASM)

pico_sdk_init()

add_executable(${PROJECT_NAME} EmbarcaTech_EstacaoMetereo_BMP-AHT 
                    ${CMAKE_CURRENT_LIST_DIR}/lib/aht20.c
                    ${CMAKE_CURRENT_LIST_DIR}/lib/bmp280.c
                    ${CMAKE_CURRENT_LIST_DIR}/lib/global_manage.c
                    ${CMAKE_CURRENT_LIST_DIR}/lib/server.c
                    ${CMAKE_CURRENT_LIST_DIR}/lib/connect_wifi.c
                    ${CMAKE_CURRENT_LIST_DIR}/lib/init_config.c
                    ${CMAKE_CURRENT_LIST_DIR}/lib/matriz.c
                    ${CMAKE_CURRENT_LIST_DIR}/lib/ssd1306.c)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio_matrix.pio)

pico_set_program_name(${PROJECT_NAME} "EmbarcaTech_EstacaoMetereo_BMP-AHT")
pico_set_program_version(${PROJECT_NAME} "0.1")

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/headers
)

target_link_libraries(${PROJECT_NAME} 

        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib 
        hardware_i2c
        hardware_pwm
        hardware_pio
        hardware_clocks
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4)

pico_add_extra_outputs(${PROJECT_NAME})
