# create executable
add_library(SimDriverInjection
        Interface.cpp
        )

# include directories
target_include_directories(SimDriverInjection PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        )
