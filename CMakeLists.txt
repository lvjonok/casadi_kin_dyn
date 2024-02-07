cmake_minimum_required(VERSION 3.17)

# Project properties
set(PROJECT_ORG lvjonok)
set(PROJECT_NAME casadi-kin-dyn)
set(PROJECT_DESCRIPTION "This is an example project, to show how to use cmeel")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR casadi_kin_dyn)

include(FetchContent)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/third_party/jrl-cmakemodules")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/third_party/jrl-cmakemodules/base.cmake")
  message(STATUS "JRL cmakemodules not found. Let's fetch it.")
  FetchContent_Declare(
    "jrl-cmakemodules"
    GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
  FetchContent_MakeAvailable("jrl-cmakemodules")
  FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(14 ENFORCE)

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  set(PYTHON_COMPONENTS Interpreter Development.Module)
  findpython()
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif()

find_package(pybind11 QUIET)

if(NOT pybind11_FOUND)
  FetchContent_Declare("pybind11"
                       GIT_REPOSITORY "https://github.com/pybind/pybind11.git")
  FetchContent_MakeAvailable("pybind11")
endif()

add_definitions(-w)

# Check requirements
find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(tinyxml REQUIRED)

# Main Library
include_directories(
  include/
  src/
  ${EIGEN3_INCLUDE_DIR}/../urdfdom_headers
  ${EIGEN3_INCLUDE_DIR}/../urdfdom
  ${EIGEN3_INCLUDE_DIR}/..
  ${EIGEN3_INCLUDE_DIR}
)

set(${PROJECT_NAME}_HEADERS include/${HEADER_DIR}/casadi_kin_dyn.h include/${HEADER_DIR}/collision_handler.h)

set(${PROJECT_NAME}_SOURCES src/casadi_kin_dyn.cpp src/collision_handler.cpp)

add_library(
  ${PROJECT_NAME} 
  SHARED 
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_HEADERS}
)

# target_link_libraries(
#   ${PROJECT_NAME} 
#   PUBLIC
# )

target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC 
  $<INSTALL_INTERFACE:include>
)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(bindings/python)
endif()

# Installation
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-error)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${CMAKE_INSTALL_LIBDIR})