# add custom target
set(IG_RESOURCE_TEST_DIR ${PROJECT_SOURCE_DIR}/resources/vehicle_interface)
set(IG_EXEC_TEST_DIR ${PROJECT_SOURCE_DIR}/lib/VehicleModel)
add_custom_target(generate_vehicle_interface node ${IG_NODE_DIR}/generator.js ${IG_RESOURCE_TEST_DIR}/config.json
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        BYPRODUCTS
            ${IG_EXEC_TEST_DIR}/VehicleModelI.h
            ${IG_EXEC_TEST_DIR}/vehicle_model_data.h
            ${IG_EXEC_TEST_DIR}/vehicle_model_data.cpp
        SOURCES
            ${IG_RESOURCE_TEST_DIR}/config.json
            ${IG_RESOURCE_TEST_DIR}/VehicleData.cpp.mustache
            ${IG_RESOURCE_TEST_DIR}/VehicleData.h.mustache
            ${IG_RESOURCE_TEST_DIR}/VehicleInterface.h.mustache
        COMMENT "Generating vehicle interface"
        )


set(SOURCE_FILES
        PrimaryController.cpp
        VehicleModel.cpp
        )

# define target
add_library(vehicle_model STATIC ${SOURCE_FILES})