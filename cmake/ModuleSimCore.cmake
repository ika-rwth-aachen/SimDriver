# message
message(STATUS "Loading SimCore library")

# set options
set(BUILD_TESTING OFF)
set(ENABLE_COVERAGE OFF)
set(CREATE_DOXYGEN_TARGET OFF)
set(GTEST_BUILD_LIBRARY OFF)
set(SIMCORE_ENABLE_SOCKET OFF)
set(SIMCORE_ENABLE_CUCUMBER OFF)

# add code
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/SimCore)