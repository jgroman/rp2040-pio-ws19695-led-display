cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME ws19695-pio-demo)
set(CMAKE_BUILD_TYPE Debug)
set(PICO_BOARD pico_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(${PROJECT_NAME} C CXX ASM)

pico_sdk_init()

add_subdirectory(lib/ws19695-pio)

add_subdirectory(src)
