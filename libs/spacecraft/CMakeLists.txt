add_library(spacecraft STATIC
    src/SpacecraftModel.cpp
    src/ManifestLoader.cpp
    src/VehicleConfig.cpp
    src/Autopilot.cpp
    src/OrbitLoader.cpp
    src/Lambert.cpp
    src/Porkchop.cpp
)

target_include_directories(spacecraft
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

find_package(tomlplusplus CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

target_link_libraries(spacecraft
    PUBLIC
        glm::glm
    PRIVATE
        tomlplusplus::tomlplusplus
        astro
)

target_compile_features(spacecraft PUBLIC cxx_std_23)
