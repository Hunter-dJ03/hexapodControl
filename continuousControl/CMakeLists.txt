cmake_minimum_required(VERSION 3.10)
project(HexapodControl)

set(CMAKE_CXX_STANDARD 14)

# Define the simulator flag
option(USE_SIMULATOR "Enable simulator support" ON)

# Pass the flag to the compiler
if(USE_SIMULATOR)
    add_definitions(-DUSE_SIMULATOR)
    # Set the CMAKE_PREFIX_PATH if the simulator is enabled
    set(CMAKE_PREFIX_PATH "/home/hunter/raisim_ws/repos/raisimlib/raisim/linux")
endif()

# Find packages
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PythonLibs 3.10 REQUIRED)  # Change Python version as needed

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

# Add executables
add_executable(continuousControl
    continuousControl.cpp
    ../modules/custom/arduinoConnection/arduinoController.cpp
    hexapod.cpp
    ../modules/custom/rsTimedLoop/rsTimedLoop.cpp
    ../modules/custom/utilities/utils.cpp
    ../modules/custom/getController/getController.cpp
)

# Conditionally add simulator sources
if(USE_SIMULATOR)
    target_sources(continuousControl PRIVATE ../modules/custom/raisimSimulatorFull/raisimSimulator.cpp)
    find_package(raisim CONFIG REQUIRED)  # Example for raisim simulator
    target_link_libraries(continuousControl PUBLIC raisim::raisim)
endif()

# Link libraries to executables
target_link_libraries(continuousControl PUBLIC 
    fmt::fmt
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
)

if(NOT WIN32)
  target_link_libraries(continuousControl PUBLIC pthread)
else()
  target_link_libraries(continuousControl PUBLIC Ws2_32 Winmm)  
endif()

# Include directories for this specific target
target_include_directories(continuousControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Copy models directory
file(COPY "../modules/models" DESTINATION ${CMAKE_BINARY_DIR})

# cmake .. -DCMAKE_PREFIX_PATH=/home/hunter/raisim_ws/repos/raisimlib/raisim/linux
