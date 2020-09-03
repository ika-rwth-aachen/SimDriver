# ------------------------------------------------------------------------------
# COVERAGE
# ------------------------------------------------------------------------------
# from: https://blog.jetbrains.com/clion/2019/10/clion-2019-3-eap-coverage-cmake-defaults-lldb/
# TODO: set this automatically when cmake mode debug is set

# option
option(ENABLE_COVERAGE "Enables the coverage check of the module" OFF)

if (ENABLE_COVERAGE)

    # message
    message(STATUS "Coverage option is enabled")

    # gnu or clang
    if (CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "--coverage")
    elseif ("${CMAKE_C_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang"
            OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
        set(CMAKE_CXX_FLAGS "-fprofile-instr-generate -fcoverage-mapping")
    endif ()

endif (ENABLE_COVERAGE)