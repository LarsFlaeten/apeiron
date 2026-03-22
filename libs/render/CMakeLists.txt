add_library(render STATIC
    src/render.cpp
)

target_include_directories(render
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(render PUBLIC cxx_std_23)

# Vulkan will be added here when the render layer is implemented:
# find_package(Vulkan REQUIRED)
# target_link_libraries(render PUBLIC Vulkan::Vulkan)
