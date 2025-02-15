cmake_minimum_required(VERSION 3.0)

# Project name
project(RobotSimulation)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(SMAKE_CXX_STANDARD_REQUIRED True)

# Add the source files
set(SOURCES
    force_calculator.cpp
    motor.cpp
    robot.cpp
    simulation.cpp
    main.cpp
)

# Add the header files
# set(HEADERS
#     force_calculator.h
#     robot.h
#     simulation.h
#     params.h
#     motor.h
# )

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

include_directories(${CMAKE_SOURCE_DIR})