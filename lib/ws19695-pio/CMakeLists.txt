add_library(ws19695_pio INTERFACE)
target_sources(ws19695_pio INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/ws19695_pio.c
)

# Assemble pio code, generate ws19695.pio.h header file
pico_generate_pio_header(ws19695_pio
    ${CMAKE_CURRENT_LIST_DIR}/ws19695.pio
)

target_link_libraries(ws19695_pio INTERFACE
    pico_stdlib
    hardware_pio
    hardware_dma
)

# add the `binary` directory so that the generated headers are included in the project
target_include_directories(ws19695_pio INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
