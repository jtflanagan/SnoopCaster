cmake_minimum_required(VERSION 3.16)

# Pull in SDK (must be before project)
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
endif()

include(cmake/pico_sdk_import.cmake)

project(snoopcaster)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Set ethernet chip
set(WIZNET_CHIP W5500)
add_definitions(-D_WIZCHIP_=W5500)

message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")


# Add libraries in subdirectories
#add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)

# Set compile options
add_compile_options(
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_executable(snoopcaster)

pico_generate_pio_header(snoopcaster
    ${CMAKE_CURRENT_SOURCE_DIR}/abus.pio)

target_sources(snoopcaster PUBLIC
    main.c
    abus.c
    buffers.c
    eth.c
)
target_include_directories(snoopcaster PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(snoopcaster PUBLIC
    pico_multicore
    pico_stdlib
    hardware_spi
    hardware_dma
    hardware_pio
)

pico_enable_stdio_usb(snoopcaster 1)
pico_enable_stdio_uart(snoopcaster 0)

pico_add_extra_outputs(snoopcaster)
