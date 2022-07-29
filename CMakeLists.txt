cmake_minimum_required(VERSION 3.14)

add_compile_options(-Wall -Wextra -Werror -Wfatal-errors)

project(oakd_logger VERSION 0.1)

# Add pybind
add_subdirectory(pybind11)

# Build with symbols
set(CMAKE_BUILD_TYPE Debug)
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# Add executable
set(SOURCES
    src/logger.cpp
    src/bindings.cpp)

include_directories(include
                    pybind11/include)

# Add Pybind
pybind11_add_module(${PROJECT_NAME} ${SOURCES})
