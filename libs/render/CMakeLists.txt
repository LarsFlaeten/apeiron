find_package(Vulkan REQUIRED)
find_package(glfw3  CONFIG REQUIRED)
find_package(vk-bootstrap CONFIG REQUIRED)
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
find_package(Stb REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG QUIET)
if(NOT TARGET glm::glm)
    if(TARGET glm)
        add_library(glm::glm ALIAS glm)
    endif()
endif()

add_library(render STATIC
    src/Context.cpp
    src/Swapchain.cpp
    src/Pipeline.cpp
    src/TonemapPipeline.cpp
    src/StarField.cpp
    src/Renderer.cpp
    src/GpuAllocator.cpp
    src/Buffer.cpp
    src/Mesh.cpp
    src/Camera.cpp
    src/Image.cpp
    src/Geometry.cpp
    src/Texture.cpp
)

target_include_directories(render
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(render
    PUBLIC
        Vulkan::Vulkan
        glfw
        vk-bootstrap::vk-bootstrap
        GPUOpen::VulkanMemoryAllocator
        glm::glm
        imgui::imgui
)

target_include_directories(render PRIVATE ${Stb_INCLUDE_DIR})

target_compile_features(render PUBLIC cxx_std_23)

# Vulkan NDC depth is [0, 1]; GLM defaults to OpenGL [-1, 1].
# This flag makes glm::perspective (and friends) emit the correct matrix.
target_compile_definitions(render PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)
