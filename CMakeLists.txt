cmake_minimum_required(VERSION 3.14)
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../build_opencv")

add_compile_options(-Wall -Wextra -Werror -Wfatal-errors)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)

project(OAKDLogger VERSION 0.1)

# Add glog
add_subdirectory(glog)

# Add utilities
add_subdirectory(utilities)

# Dependencies (optional, only used for example)
find_package(OpenCV REQUIRED)

# Build with symbols
set(CMAKE_BUILD_TYPE Debug)
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# Add executable
set(SOURCES
    src/logger.cpp
    src/bindings.cpp)

# Add Pybind
pybind11_add_module(${PROJECT_NAME} ${SOURCES})

include_directories(include
                    pybind11/include
                    utilities/magic_enum/include
                    utilities/include
                    )

# Link with libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    depthai::opencv
    ${OpenCV_LIBS} # optional, used for example
    glog::glog
    utilities
    )

target_compile_options(${PROJECT_NAME} PUBLIC -fPIC)
add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")


