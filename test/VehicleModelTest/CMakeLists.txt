# create executable
add_executable(VehicleModelTest
        CommunicationTest.cpp
        VehicleModelTest.cpp
        )

# boost
find_package(Boost 1.40.0 REQUIRED system)

# link libs
target_link_libraries(VehicleModelTest PRIVATE
        vehicle_model
        )

# include directories
target_include_directories(VehicleModelTest PRIVATE
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/lib/VehicleModel/src
        )

# add test
add_gtest(VehicleModelTest)