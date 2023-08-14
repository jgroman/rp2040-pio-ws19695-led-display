
add_executable(${PROJECT_NAME}
    main.c
    u8x8_d_ws19695.c
)

pico_generate_pio_header(${PROJECT_NAME} 
    ${CMAKE_CURRENT_LIST_DIR}/ws19695.pio
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_pwm
    hardware_dma
    u8g2
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT_NAME}
        pico_cyw43_arch_none
    )
endif()

pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_add_extra_outputs(${PROJECT_NAME})
