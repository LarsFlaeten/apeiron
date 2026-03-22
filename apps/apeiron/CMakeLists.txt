# ---------------------------------------------------------------------------
# Shader compilation  (GLSL → SPIR-V at build time)
# ---------------------------------------------------------------------------
find_program(GLSLC_EXECUTABLE glslc REQUIRED
    DOC "glslc shader compiler from the Vulkan SDK")

set(SHADER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set(SHADER_OUT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders")
file(MAKE_DIRECTORY "${SHADER_OUT_DIR}")

set(SHADER_SPIRV)
foreach(SHADER triangle.vert triangle.frag)
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

add_executable(apeiron
    main.cpp
    ScenarioConfig.cpp
)

add_dependencies(apeiron apeiron_shaders)

target_link_libraries(apeiron
    PRIVATE
        universe
        render
        tomlplusplus::tomlplusplus
)

# Bake absolute paths into the binary (avoids working-directory sensitivity).
target_compile_definitions(apeiron
    PRIVATE
        APEIRON_SHADER_DIR="${SHADER_OUT_DIR}"
        APEIRON_SCENARIO_FILE="${CMAKE_CURRENT_SOURCE_DIR}/data/scenario.toml"
)

target_compile_features(apeiron PRIVATE cxx_std_23)
