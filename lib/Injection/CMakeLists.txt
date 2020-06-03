# create executable
add_library(injection
        src/InjectionInterface.cpp
        )

# include directories
target_include_directories(injection PRIVATE
        include/injection
        )