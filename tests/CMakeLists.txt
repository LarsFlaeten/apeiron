find_package(Catch2 3 QUIET)

if(Catch2_FOUND)
    # Tests will be added here as the libraries grow.
    # Example:
    # add_executable(universe_tests test_universe.cpp)
    # target_link_libraries(universe_tests PRIVATE Catch2::Catch2WithMain universe)
    # catch_discover_tests(universe_tests)
else()
    message(STATUS "Catch2 not found — skipping tests. Install via vcpkg to enable.")
endif()
