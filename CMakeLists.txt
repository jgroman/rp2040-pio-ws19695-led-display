cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME ws19695-pio)
set(CMAKE_BUILD_TYPE Debug)
set(PICO_BOARD pico_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(${PROJECT_NAME} C CXX ASM)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    main.c
)

pico_generate_pio_header(${PROJECT_NAME} 
    ${CMAKE_CURRENT_LIST_DIR}/ws19695.pio
)

pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_add_extra_outputs(${PROJECT_NAME})


target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_pwm
    hardware_dma
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT_NAME}
        pico_cyw43_arch_none
    )
endif()
