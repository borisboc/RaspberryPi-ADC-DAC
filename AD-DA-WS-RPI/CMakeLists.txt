cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ADDA_WS_RPI)

#message("current dir")
#message(${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

find_package (bcm2835 REQUIRED ${CMAKE_MODULE_PATH})

add_library(ADDA_WS_RPI AD-DA-WS-RPI.c)

include_directories(${BCM2835_INCLUDE_DIRS})
target_link_libraries (ADDA_WS_RPI ${BCM2835_LIBRARIES})
