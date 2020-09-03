# documentation
if (CREATE_DOXYGEN_TARGET)

    # message
    message(STATUS "Generation of doxygen target enabled")

    # Require dot, treat the other components as optional
    find_package(Doxygen
            REQUIRED dot
            OPTIONAL_COMPONENTS mscgen dia)

    if (DOXYGEN_FOUND)

        # create doc directory
        file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)

        # settings
        set(DOXYGEN_GENERATE_HTML YES)
        set(DOXYGEN_GENERATE_MAN YES)
        set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
        set(DOXYGEN_EXCLUDE_PATTERNS AgentModelInjection.*)
        set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

        # create target
        doxygen_add_docs(
                doxygen
                ${PROJECT_SOURCE_DIR}/src README.md
                COMMENT "Generate man pages"
        )

    endif (DOXYGEN_FOUND)

    # unset doxygen target
    set(CREATE_DOXYGEN_TARGET OFF CACHE BOOL "disabled doxygen target for submodules" FORCE)

endif (CREATE_DOXYGEN_TARGET)