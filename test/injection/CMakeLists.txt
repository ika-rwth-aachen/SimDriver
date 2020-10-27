# create executable
add_executable(InjectionTest
        InjectionTest.cpp
        )

# include directories
target_include_directories(InjectionTest PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        )

# link library
target_link_libraries(InjectionTest PRIVATE
        SimDriverInjection
        )

# add gtest
add_gtest(InjectionTest)