cmake_minimum_required(VERSION 3.25)

project(apeiron
    VERSION 0.1.0
    DESCRIPTION "Physics-correct, unlimited space simulator"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(libs/astro)
add_subdirectory(libs/universe)
add_subdirectory(libs/render)
add_subdirectory(libs/spacecraft)

add_subdirectory(apps/apeiron)

enable_testing()
add_subdirectory(tests)
