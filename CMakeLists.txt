cmake_minimum_required(VERSION 3.5)

# the project
project(SimDriver VERSION 0.1)
set(CMAKE_CXX_STANDARD 14)

# options
option(BUILD_TESTS "Builds the test executables if set to ON." OFF)
option(BUILD_GTEST "Sets or unsets the option to build gtest" OFF)
option(BUILD_FOR_COVERAGE "Builds the project with coverage test options." OFF)
option(CREATE_INTERFACE_GENERATOR_TARGET "Creates the interface generator target if set." OFF)
option(BUILD_INTERFACE_GENERATOR_TESTS "Builds the interface generator tests if set." OFF)
option(CREATE_DOXYGEN_TARGET "Creates the doxygen documentation if set." OFF)
option(BUILD_EXECUTABLE_CAR_MAKER "Builds the CarMaker plugin." OFF)
option(BUILD_VEHICLE_MODEL "Builds the CarMaker plugin." OFF)
option(BUILD_SIMULATION_ENVIRONMENT "Builds the CarMaker plugin." OFF)
option(BUILD_WITH_INJECTION "Building the agent model with injection functionality." ON)

# add ./cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# set interface generator dir
set(IG_NODE_DIR ${PROJECT_SOURCE_DIR}/lib/SimCore/generator/src)

# code coverage
if (BUILD_FOR_COVERAGE)

    # message
    message("-- Coverage option is enabled")

    # gnu or clang
    if (CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "--coverage")
    elseif ("${CMAKE_C_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang"
            OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
        set(CMAKE_CXX_FLAGS "-fprofile-instr-generate -fcoverage-mapping")
    endif ()

    # from: https://blog.jetbrains.com/clion/2019/10/clion-2019-3-eap-coverage-cmake-defaults-lldb/

endif (BUILD_FOR_COVERAGE)


# documentation
if (CREATE_DOXYGEN_TARGET)

    # message
    message("-- Generation of doxygen target enabled")

    # Require dot, treat the other components as optional
    find_package(Doxygen
            REQUIRED dot
            OPTIONAL_COMPONENTS mscgen dia)

    if (DOXYGEN_FOUND)

        # create doc directory
        file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)

        # settings
        set(DOXYGEN_GENERATE_HTML YES)
        set(DOXYGEN_GENERATE_MAN YES)
        set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
        set(DOXYGEN_EXCLUDE_PATTERNS AgentModelInjection.*)
        set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

        # create target
        doxygen_add_docs(
                doxygen
                ${PROJECT_SOURCE_DIR}/src README.md
                COMMENT "Generate man pages"
        )

    endif (DOXYGEN_FOUND)

    # unset doxygen target
    set(CREATE_DOXYGEN_TARGET OFF CACHE BOOL "disabled doxygen target for submodules" FORCE)

endif (CREATE_DOXYGEN_TARGET)


# injection
if(BUILD_WITH_INJECTION)

    # add injection sources
    add_subdirectory(lib/Injection)

    # add definition
    add_definitions(-DWITH_INJECTION=true)

endif(BUILD_WITH_INJECTION)


# library code
add_subdirectory(src/)

# testing
if (BUILD_TESTS)

    # message
    message("-- Testing of agent model enabled")

    # enable testing
    enable_testing()

    # switch (build or use installed lib)
    if(BUILD_GTEST)

        # add subdirectory
        add_subdirectory(googletest)
        set(GTEST_BOTH_LIBRARIES gtest_main)
        include_directories(${PROJECT_SOURCE_DIR}/googletest/googletest/include)

    else()

        # find gtest
        find_package(GTest REQUIRED)
        include_directories(${GTEST_INCLUDE_DIRS})

    endif()

    # load macros for tests
    include(AddGoogleTest)

    # setting (traffic simulation)
    set(BUILD_TRAFFIC_SIMULATION ON CACHE BOOL "enable traffic simulation target" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "disable tests" FORCE)
    set(BUILD_GTEST OFF CACHE BOOL "disable gtest" FORCE)
    set(BUILD_FOR_COVERAGE OFF CACHE BOOL "disable coverage" FORCE)

    # setting (vehicle model and simulation environment)
    set(BUILD_VEHICLE_MODEL ON CACHE BOOL "enable vehicle model target" FORCE)
    set(BUILD_SIMULATION_ENVIRONMENT ON CACHE BOOL "enable simulation environment target" FORCE)

    # json
    include_directories(lib/SimCore/lib/json/include)

    # necessary libs
    add_subdirectory(lib/SimCore)
    add_subdirectory(lib/SimMap)

    # set parameters
    set(SIMCORE_INCLUDES ${PROJECT_SOURCE_DIR}/lib/SimCore/include)
    set(SIMMAP_INCLUDES ${PROJECT_SOURCE_DIR}/lib/SimMap/include)

    # add test folder
    add_subdirectory(test/functions)
    add_subdirectory(test/vehicle_model)
    add_subdirectory(test/capabilities)
    add_subdirectory(test/traffic)

    message(${SIMMAP_INCLUDES})

endif (BUILD_TESTS)


if(BUILD_EXECUTABLE_CAR_MAKER)

    message("-- Building CarMaker plugin enabled.")

    # setting
    set(BUILD_VEHICLE_MODEL ON CACHE BOOL "enable vehicle model target" FORCE)

    # create bin dir
    set(BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
    file(MAKE_DIRECTORY ${BIN_DIR})

    # add directory
    add_subdirectory(${PROJECT_SOURCE_DIR}/app/CarMakerEgo)
    add_subdirectory(${PROJECT_SOURCE_DIR}/app/CarMakerTarget)


endif(BUILD_EXECUTABLE_CAR_MAKER)


if(BUILD_VEHICLE_MODEL)

    message("-- Building vehicle model.")

    # add vehicle model
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/VehicleModel)

endif(BUILD_VEHICLE_MODEL)


if(BUILD_SIMULATION_ENVIRONMENT)

    message("-- Building simulation environment.")

    # add vehicle model
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/Simulation)

endif(BUILD_SIMULATION_ENVIRONMENT)


# interface generator
if(CREATE_INTERFACE_GENERATOR_TARGET)

    message("-- Building interface generator target enabled")

    # set some variables
    set(IG_EXEC_DIR ${PROJECT_SOURCE_DIR}/src)
    set(IG_RESOURCE_DIR ${PROJECT_SOURCE_DIR}/resources/interface)

    # init node process
    execute_process(COMMAND npm install
            WORKING_DIRECTORY ${IG_NODE_DIR}
            ERROR_FILE ${IG_NODE_DIR}/error_install.txt
            OUTPUT_FILE ${IG_NODE_DIR}/error_install.txt
        )

    # add custom target
    add_custom_target(generate_interface node ${IG_NODE_DIR}/generator.js ${IG_RESOURCE_DIR}/config.json
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            BYPRODUCTS
                ${IG_EXEC_DIR}/Interface.h
                ${IG_EXEC_DIR}/AgentModelInjection.h
                ${IG_EXEC_DIR}/AgentModelInjection.cpp
            SOURCES
                ${IG_RESOURCE_DIR}/fields.csv
                ${IG_RESOURCE_DIR}/config.json
                ${IG_RESOURCE_DIR}/Injection.cpp.mustache
                ${IG_RESOURCE_DIR}/Injection.h.mustache
                ${IG_RESOURCE_DIR}/Interface.h.mustache
            COMMENT "Generating agent model interface"
        )


endif(CREATE_INTERFACE_GENERATOR_TARGET)
