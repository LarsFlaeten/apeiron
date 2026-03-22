add_library(universe STATIC
    src/universe.cpp
)

target_include_directories(universe
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(universe PUBLIC cxx_std_23)
