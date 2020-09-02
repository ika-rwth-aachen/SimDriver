cmake_minimum_required(VERSION 3.5)

# the project
project(SimDriver VERSION 0.1)
set(CMAKE_CXX_STANDARD 14)

# options
option(CREATE_DOXYGEN_TARGET "Creates the doxygen documentation if set." OFF)
option(BUILD_WITH_INJECTION "Building the agent model with injection functionality." ON)


# add ./cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# load modules
include(ModuleCoverage)
include(ModuleGtest)


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
