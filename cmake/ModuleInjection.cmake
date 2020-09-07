# injection
if(BUILD_WITH_INJECTION)

    # message
    message(STATUS "Injection option is enabled")

    # add injection sources
    add_subdirectory(lib/Injection)

    # add definition
    add_definitions(-DWITH_INJECTION=true)

endif(BUILD_WITH_INJECTION)