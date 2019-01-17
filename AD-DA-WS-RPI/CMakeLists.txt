cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(ADDA_WS_RPI)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")

list(APPEND CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

SET(LIB_FILES AD-DA-WS-RPI.c)

if(MOCK_SPI)
    message(WARNING "CAUTION : mocking SPI communication")
    SET(LIB_FILES ${LIB_FILES} AD-DA-MOCKSPI.c)
else()
    if(USE_BCM)
        message("Using BCM2835 as SPI library")
        find_package(bcm2835 REQUIRED ${CMAKE_MODULE_PATH})
        include_directories(${BCM2835_INCLUDE_DIRS})
        SET(LIB_FILES ${LIB_FILES} AD-DA-BCM.c)
    else()
        message(FATAL_ERROR "You must choose a SPI library (e.g. BCM2835)")
    endif()
endif()

add_library(ADDA_WS_RPI ${LIB_FILES})

if(USE_BCM)
    target_link_libraries (ADDA_WS_RPI ${BCM2835_LIBRARIES})
endif()
