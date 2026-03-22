add_executable(apeiron
    main.cpp
)

target_link_libraries(apeiron
    PRIVATE
        universe
        render
)

target_compile_features(apeiron PRIVATE cxx_std_23)
