
# define target
add_library(SimDriver STATIC
        Behavior.cpp
        Controller.cpp
        model_collection.cpp
        )

# include directories
target_include_directories(SimDriver PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        )