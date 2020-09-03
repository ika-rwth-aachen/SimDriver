if (BUILD_VEHICLE_MODEL)

    # message
    message(STATUS "Vehicle model option is enables")

    # set a few variables
    set(VEHICLE_MODEL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/VehicleModel/src)

    # add subdirectory
    add_subdirectory(${VEHICLE_MODEL_INCLUDE_DIR})

endif (BUILD_VEHICLE_MODEL)