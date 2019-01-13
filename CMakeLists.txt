cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(TestingAdda)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

add_subdirectory(AD-DA-WS-RPI)

add_executable(testAdda mainTestAdda.c)
target_link_libraries(testAdda ADDA_WS_RPI)

