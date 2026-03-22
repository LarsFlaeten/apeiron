add_library(universe STATIC
    src/KernelPool.cpp
    src/Ephemeris.cpp
)

target_include_directories(universe
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Pull in CSPICE headers (for spkezr_c etc.) without re-linking the static lib —
# all CSPICE symbols are already embedded in libastro.so.
get_target_property(_cspice_inc cspice INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(universe PRIVATE ${_cspice_inc})

target_link_libraries(universe
    PUBLIC
        astro
)

target_compile_features(universe PUBLIC cxx_std_23)
