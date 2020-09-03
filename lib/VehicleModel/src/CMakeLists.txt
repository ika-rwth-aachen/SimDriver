# sources
set(SOURCE_FILES
        PrimaryController.cpp
        VehicleModel.cpp
        )

# define target
add_library(vehicle_model STATIC ${SOURCE_FILES})