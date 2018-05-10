cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(TestingAdda)

add_subdirectory(AD-DA-WS-RPI)

add_executable(testAdda mainTestAdda.c)
target_link_libraries(testAdda ADDA_WS_RPI)

