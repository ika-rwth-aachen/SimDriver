add_subdirectory(simdriver)

# add injection if option is set
if(BUILD_WITH_INJECTION)
    add_subdirectory(injection)
endif(BUILD_WITH_INJECTION)
