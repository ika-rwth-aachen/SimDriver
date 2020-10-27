cmake_minimum_required(VERSION 3.5)

# the project
project(SimDriver VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)

# add ./cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# options
option(CREATE_DOXYGEN_TARGET "Creates the doxygen documentation if set." OFF)
option(BUILD_TESTING "Building the tests of the driver model." OFF)
option(BUILD_GTEST_LIBRARY "Enables gtest to be built from the submodule" OFF)
option(ENABLE_COVERAGE "Builds the code with code coverage functionality." OFF)
option(BUILD_WITH_INJECTION "Builds the agent model with injection functionality." ON)

# load modules
include(ModuleDoxygen)
include(ModuleCoverage)
include(ModuleGtest)
include(ModuleInjection)

# library code
add_subdirectory(src/)
