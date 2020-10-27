# create executable
add_executable(CapabilitiesTests
        AMT_Pedal.cpp
#        AMT_Acceleration.cpp
#        AMT_PredictionAcceleration.cpp
#        AMT_PredictionAccelerationAll.cpp
#        AMT_Steering.cpp
#        AMT_LateralControl.cpp
#        AMT_HorizonControl.cpp
#        AMT_Following.cpp
#        AMT_SpeedControl.cpp
#        AMT_StopSigns.cpp
#        AMT_Longitudinal.cpp
        )

# include directories
target_include_directories(CapabilitiesTests PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/lib/SimCore/include
        )

# link
target_link_libraries(CapabilitiesTests PRIVATE
        agent_model
        vehicle_model
        traffic_simulation
        )

# add gtest
add_gtest(CapabilitiesTests)