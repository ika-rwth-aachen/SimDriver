# message
message(STATUS "Loading SimCore library")

# set options
set(BUILD_TESTING OFF)
set(ENABLE_COVERAGE OFF)
set(CREATE_DOXYGEN_TARGET OFF)
set(REMOTE_FUNCTIONALITY OFF)
set(GTEST_BUILD_LIBRARY OFF)

# add code
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/SimCore)