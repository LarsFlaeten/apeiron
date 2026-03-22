find_package(Vulkan REQUIRED)
find_package(glfw3  CONFIG REQUIRED)
find_package(vk-bootstrap CONFIG REQUIRED)
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
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
    src/Renderer.cpp
    src/GpuAllocator.cpp
    src/Buffer.cpp
    src/Mesh.cpp
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
)

target_compile_features(render PUBLIC cxx_std_23)
