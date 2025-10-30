cmake_minimum_required(VERSION 3.15)
project(GridWorldEnvUtils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0148 NEW)

include(FetchContent)


find_package(pybind11 QUIET)

if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found, fetching with FetchContent...")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.12.0 
    )
    FetchContent_MakeAvailable(pybind11)
endif()


# Eigen3
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found, fetching with FetchContent...")
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0 
    )
    FetchContent_MakeAvailable(eigen)
endif()

add_library(gridutils
    src/gridworldenvutils.cpp
    src/AStar.cpp
)

target_link_libraries(gridutils PUBLIC Eigen3::Eigen)

pybind11_add_module(_gridworldenvutils
    src/binder.cpp
)

target_link_libraries(_gridworldenvutils PRIVATE gridutils)

target_include_directories(_gridworldenvutils PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

install(TARGETS _gridworldenvutils
    DESTINATION gridworldenvutils
)




