# add test folders
add_subdirectory(utils)
add_subdirectory(models)
add_subdirectory(capabilities)

if(BUILD_WITH_INJECTION)
    add_subdirectory(injection)
endif(BUILD_WITH_INJECTION)