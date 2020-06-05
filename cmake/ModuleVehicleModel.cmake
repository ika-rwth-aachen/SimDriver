# option (to activate or deactivate the vehicle model)
option(SIMDRIVER_BUILD_VEHICLE_MODEL "Building the integrated vehicle model feature (used e.g. for testing)." OFF)

# TODO: check if test is on but vehicle model is off -> error or autoset

if (SIMDRIVER_BUILD_VEHICLE_MODEL)

    # message
    message(STATUS "Building vehicle model")

    # add subdirectory
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/VehicleModel)

endif (SIMDRIVER_BUILD_VEHICLE_MODEL)