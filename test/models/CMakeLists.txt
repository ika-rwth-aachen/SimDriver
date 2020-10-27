# create executable
add_executable(ModelsTest
        FollowingTargets.cpp
        LaneChangeDecision.cpp
        SalvucciAndGray.cpp
        VelocityControl.cpp
        )

# include directories
target_include_directories(ModelsTest PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        )

# link library
target_link_libraries(ModelsTest PRIVATE
        SimDriver
        )

# add gtest
add_gtest(ModelsTest)