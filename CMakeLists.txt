cmake_minimum_required(VERSION 3.10)
project(franka_pybind)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_PREFIX_PATH "/home/surgicalai/.conda/envs/franka_5/usr/local/lib/cmake/Franka")
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

SET(PRJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}/../")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")
SET(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/pyfranka/")

aux_source_directory(./src FRANKA_PYBIND_ROOT_SRCS)

add_subdirectory(pybind11)

pybind11_add_module(franka_pybind ${FRANKA_PYBIND_ROOT_SRCS})

target_link_libraries(franka_pybind PUBLIC Franka::Franka)

