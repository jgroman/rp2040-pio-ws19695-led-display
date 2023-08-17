
add_executable(${PROJECT_NAME}
    main.c
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    ws19695_pio
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT_NAME}
        pico_cyw43_arch_none
    )
endif()

pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_add_extra_outputs(${PROJECT_NAME})
