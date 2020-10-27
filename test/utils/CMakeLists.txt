# create executable
add_executable(UtilsTest
        HelperFunctionsTest.cpp
        VelocityHorizonTest.cpp
        FilterTest.cpp
        StopHorizonTest.cpp
        DistanceTimeIntervalTest.cpp
        )

# include directories
target_include_directories(UtilsTest PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        )

# add gtest
add_gtest(UtilsTest)