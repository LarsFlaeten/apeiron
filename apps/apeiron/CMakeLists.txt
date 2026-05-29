# ---------------------------------------------------------------------------
# Shader compilation  (GLSL → SPIR-V at build time)
# ---------------------------------------------------------------------------
find_program(GLSLC_EXECUTABLE glslc REQUIRED
    DOC "glslc shader compiler from the Vulkan SDK")

set(SHADER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set(SHADER_OUT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders")
file(MAKE_DIRECTORY "${SHADER_OUT_DIR}")

set(SHADER_SPIRV)
foreach(SHADER triangle.vert triangle.tesc triangle.tese triangle.frag tonemap.vert tonemap.frag star.vert star.frag bloom_threshold.frag bloom_blur.frag ring.vert ring.frag atmosphere_transmittance.comp atmosphere.vert atmosphere.frag mesh.vert mesh.frag plume.vert plume.frag)
    add_custom_command(
        OUTPUT  "${SHADER_OUT_DIR}/${SHADER}.spv"
        COMMAND ${GLSLC_EXECUTABLE}
                "${SHADER_SRC_DIR}/${SHADER}"
                -o "${SHADER_OUT_DIR}/${SHADER}.spv"
        DEPENDS "${SHADER_SRC_DIR}/${SHADER}"
        COMMENT "Compiling shader: ${SHADER}"
    )
    list(APPEND SHADER_SPIRV "${SHADER_OUT_DIR}/${SHADER}.spv")
endforeach()

add_custom_target(apeiron_shaders DEPENDS ${SHADER_SPIRV})

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
find_package(tomlplusplus CONFIG REQUIRED)
find_package(PkgConfig QUIET)

add_executable(apeiron
    main.cpp
    ScenarioConfig.cpp
    DockingConstraint.cpp
    MFD.cpp
    MFDMenu.cpp
    OrbitalMFD.cpp
    DockingMFD.cpp
    CamMFD.cpp
    TransferMFD.cpp
    MapMFD.cpp
    OrbitDiagram.cpp
    OffscreenCam.cpp
    NavConsole.cpp
    NavHUD.cpp
    VoiceAnnouncer.cpp
    SimSave.cpp
    OBCEventQueue.cpp
    BurnController.cpp
    OBCMFD.cpp
)

add_dependencies(apeiron apeiron_shaders)

target_link_libraries(apeiron
    PRIVATE
        universe
        render
        spacecraft
        tomlplusplus::tomlplusplus
)

# Expose SPICE headers so main.cpp can call pxform_c for star catalog loading.
get_target_property(_cspice_inc cspice INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(apeiron PRIVATE ${_cspice_inc})

# Bake absolute paths into the binary (avoids working-directory sensitivity).
target_compile_definitions(apeiron
    PRIVATE
        APEIRON_SHADER_DIR="${SHADER_OUT_DIR}"
        APEIRON_SCENARIO_FILE="${CMAKE_SOURCE_DIR}/data/scenarios/scenario.toml"
        APEIRON_STAR_CATALOG="${CMAKE_SOURCE_DIR}/data/stars/hygdata_v42.csv"
        APEIRON_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
)

target_compile_features(apeiron PRIVATE cxx_std_23)

# ---------------------------------------------------------------------------
# Optional: eSpeak NG text-to-speech (OBC voice announcements)
# ---------------------------------------------------------------------------
if (PkgConfig_FOUND)
    pkg_check_modules(ESPEAK_NG QUIET espeak-ng)
endif()
if (ESPEAK_NG_FOUND)
    target_compile_definitions(apeiron PRIVATE APEIRON_VOICE=1)
    target_include_directories(apeiron PRIVATE ${ESPEAK_NG_INCLUDE_DIRS})
    target_link_libraries(apeiron PRIVATE ${ESPEAK_NG_LINK_LIBRARIES})
    message(STATUS "eSpeak NG ${ESPEAK_NG_VERSION} found — OBC voice enabled")
else()
    message(STATUS "eSpeak NG not found — OBC voice disabled (install libespeak-ng-dev)")
endif()
