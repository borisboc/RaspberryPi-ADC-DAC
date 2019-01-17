cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(TestingAdda)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

SET(MOCK_SPI False CACHE BOOL "On : SPI communication is mocked. Off : use real SPI lib such as BCM2835")
SET(USE_BCM True CACHE BOOL "On :  use BCM2835 as SPI lib")

add_subdirectory(AD-DA-WS-RPI)

add_executable(testAdda mainTestAdda.c)
target_link_libraries(testAdda ADDA_WS_RPI)

