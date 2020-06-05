# ------------------------------------------------------------------------------
# Testing
#
# * set BUILD_LIB_GTEST whether to build gtest as submodule
# * Include this file when testing is enabled
# * Put your tests in the root/test folder
# * add a CMakeLists.txt and use add_gtest(...) macro
# ------------------------------------------------------------------------------

# options
option(SIMDRIVER_BUILD_TESTS "Building the tests for the driver model (gtest must be installed)." OFF)
option(SIMDRIVER_BUILD_GTEST_LIBRARY "Enables gtest to be built from the submodule" OFF)

if (SIMDRIVER_BUILD_TESTS)

    # define macro
    macro(add_gtest TESTNAME)

        # link library
        target_link_libraries(${TESTNAME} PRIVATE ${GTEST_BOTH_LIBRARIES} pthread)
        target_include_directories(${TESTNAME} PRIVATE ${GTEST_INCLUDE_DIRS})

        if(GOOGLE_TEST_INDIVIDUAL)
            if(CMAKE_VERSION VERSION_LESS 3.10)
                gtest_add_tests(TARGET ${TESTNAME}
                        TEST_PREFIX "${TESTNAME}."
                        TEST_LIST TmpTestList)
                set_tests_properties(${TmpTestList} PROPERTIES FOLDER "Tests")
            else()
                gtest_discover_tests(${TESTNAME}
                        TEST_PREFIX "${TESTNAME}."
                        PROPERTIES FOLDER "Tests")
            endif()
        else()
            add_test(NAME ${TESTNAME} COMMAND $<TARGET_FILE:${TESTNAME}>)
            set_target_properties(${TESTNAME} PROPERTIES FOLDER "Tests")
        endif()

    endmacro()


    # message
    message(STATUS "Testing (gtest) enabled")

    # gtest flag
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # enable testing
    enable_testing()

    # switch (build or use installed lib)
    if(SIMDRIVER_BUILD_GTEST_LIBRARY)

        # message
        message(STATUS "ATTENTION: gtest must be cloned into the lib folder (folder name gtest)")

        # add gtest sources
        add_subdirectory(lib/googletest)

        # set variables
        set(GTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/googletest/googletest/include)
        set(GTEST_BOTH_LIBRARIES gtest gtest_main)

    else()

        # find gtest
        find_package(GTest REQUIRED)

    endif(SIMDRIVER_BUILD_GTEST_LIBRARY)

    # add test folder
    add_subdirectory(${PROJECT_SOURCE_DIR}/test)

endif (SIMDRIVER_BUILD_TESTS)