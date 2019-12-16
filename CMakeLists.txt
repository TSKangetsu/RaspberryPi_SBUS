cmake_minimum_required (VERSION 3.8)

project ("RaspberryPI_SBUS")
find_library(WIRING wiringPi)

add_executable (RaspberryPI_SBUS "RaspberryPI_SBUS.cpp" "RPiSbus.h")
target_link_libraries(RaspberryPI_SBUS  ${WIRING})