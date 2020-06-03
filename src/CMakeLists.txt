# if build with injection option is on, then add injection sources
if (BUILD_WITH_INJECTION)

    # set headers and sources
    set(INJECTION_SRC AgentModelInjection.cpp)
    set(INJECTION_HDR "\;AgentModelInjection.h")

else ()

    # set headers and sources
    set(INJECTION_SRC)
    set(INJECTION_HDR)

endif (BUILD_WITH_INJECTION)


# define target
add_library(agent_model STATIC
        AgentModel.cpp
        model_collection.cpp
        ${INJECTION_SRC})


if (BUILD_WITH_INJECTION)

    target_link_libraries(agent_model PRIVATE
            injection
            )

    target_include_directories(agent_model PUBLIC
            ${PROJECT_SOURCE_DIR}/lib/Injection/include
            )

endif (BUILD_WITH_INJECTION)