# injection
if(BUILD_WITH_INJECTION)

    # message
    message(STATUS "Injection option is enabled")

    # add definition
    add_definitions(-DWITH_INJECTION=true)

endif(BUILD_WITH_INJECTION)