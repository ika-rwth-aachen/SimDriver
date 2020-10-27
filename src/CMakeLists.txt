add_subdirectory(SimDriver)

# add injection if option is set
if(BUILD_WITH_INJECTION)
    add_subdirectory(Injection)
endif(BUILD_WITH_INJECTION)
