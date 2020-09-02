# create executable
add_executable(VehicleModelTest
        VehicleModelTest.cpp
        )

# link libs
target_link_libraries(VehicleModelTest PRIVATE
        vehicle_model
        )

# include directories
target_include_directories(VehicleModelTest PRIVATE
        ${PROJECT_SOURCE_DIR}/lib
        )

# add test
add_gtest(VehicleModelTest)