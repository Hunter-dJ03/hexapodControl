cmake_minimum_required(VERSION 3.10)

# Project name
project(SteelSeriesDeviceFinder)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable and link sources
add_executable(getControllerTest getControllerTest.cpp getController.cpp)
