add_library(ws19695_pio INTERFACE)

# Prevent "region `FLASH' overflowed" build error when adding u8g2 fonts
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections -Wl,--gc-sections")

add_subdirectory(lib/u8g2)

# Assemble pio code, generate ws19695.pio.h header file
pico_generate_pio_header(ws19695_pio
    ${CMAKE_CURRENT_LIST_DIR}/ws19695.pio
)

target_sources(ws19695_pio INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/ws19695_pio.c
)

target_link_libraries(ws19695_pio INTERFACE
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_pwm
    u8g2
)

# add the `binary` directory so that the generated headers are included in the project
# target_include_directories(ws19695_pio INTERFACE
#     ${CMAKE_CURRENT_SOURCE_DIR}
#     ${CMAKE_CURRENT_BINARY_DIR}
# )
