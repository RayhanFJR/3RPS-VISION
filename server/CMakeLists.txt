cmake_minimum_required(VERSION 3.10)
project(RehabControlServer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(modbus REQUIRED)
find_package(Boost 1.66.0 REQUIRED COMPONENTS system)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Source files
set(SOURCES
    src/main_server.cpp
    
    src/trajectory/TrajectoryManager.cpp
    
    src/modbus/ModbusServer.cpp
    src/modbus/DataHandler.cpp
    
    src/serial/SerialPort.cpp
    
    src/state_machine/StateMachine.cpp
)

# Create executable
add_executable(rehab_server ${SOURCES})

# Link libraries
target_link_libraries(rehab_server PRIVATE
    modbus
    Boost::system
)

# Compiler flags
if(MSVC)
    target_compile_options(rehab_server PRIVATE /W4)
else()
    target_compile_options(rehab_server PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Install
install(TARGETS rehab_server DESTINATION bin)