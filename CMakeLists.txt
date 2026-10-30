cmake_minimum_required(VERSION 3.18)
project(franka_pybind LANGUAGES CXX)

# Supply environment-specific paths through CMake arguments or presets.
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Franka CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

option(PYFRANKA_USE_BUNDLED_PYBIND11 "Use vendored rather than installed pybind11" OFF)
if(PYFRANKA_USE_BUNDLED_PYBIND11)
  add_subdirectory(pybind11)
else()
  # Respect pybind11_DIR / CMAKE_PREFIX_PATH, then try the selected Python.
  find_package(pybind11 CONFIG QUIET)
  if(NOT pybind11_FOUND)
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
      RESULT_VARIABLE _pybind11_result
      OUTPUT_VARIABLE _pybind11_cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(_pybind11_result EQUAL 0)
      list(PREPEND CMAKE_PREFIX_PATH "${_pybind11_cmake_dir}")
    endif()
    find_package(pybind11 CONFIG REQUIRED)
  endif()
endif()

pybind11_add_module(franka_pybind
  src/franka_api.cpp
  src/franka_log.cpp
  src/franka_pybind.cpp
  src/motion_generator.cpp
)
target_link_libraries(franka_pybind PRIVATE Franka::Franka Eigen3::Eigen)

# Default output supports setup.py's editable installation.
set(PYFRANKA_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/pyfranka"
    CACHE PATH "Directory for the compiled Python extension")
set_target_properties(franka_pybind PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${PYFRANKA_OUTPUT_DIRECTORY}"
  RUNTIME_OUTPUT_DIRECTORY "${PYFRANKA_OUTPUT_DIRECTORY}"
)
foreach(_config IN LISTS CMAKE_CONFIGURATION_TYPES)
  string(TOUPPER "${_config}" _config_upper)
  set_target_properties(franka_pybind PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_${_config_upper} "${PYFRANKA_OUTPUT_DIRECTORY}"
    RUNTIME_OUTPUT_DIRECTORY_${_config_upper} "${PYFRANKA_OUTPUT_DIRECTORY}"
  )
endforeach()
