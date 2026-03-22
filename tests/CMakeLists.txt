find_package(Catch2 3 QUIET)

if(Catch2_FOUND)
    include(CTest)
    include(Catch)

    add_executable(universe_tests
        test_universe.cpp
    )

    target_link_libraries(universe_tests
        PRIVATE
            Catch2::Catch2WithMain
            universe
    )

    # Inject absolute path to data/kernels/ so tests find SPICE kernels
    # regardless of which directory ctest is invoked from.
    target_compile_definitions(universe_tests
        PRIVATE
            APEIRON_KERNEL_DIR="${CMAKE_SOURCE_DIR}/data/kernels"
    )

    catch_discover_tests(universe_tests)

else()
    message(STATUS "Catch2 not found — skipping tests. Install via vcpkg to enable.")
endif()
