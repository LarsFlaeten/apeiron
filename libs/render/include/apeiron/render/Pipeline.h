#pragma once

#include <vulkan/vulkan.hpp>
#include <filesystem>

namespace apeiron::render {

class Context;
class Swapchain;

// Owns the render pass, pipeline layout, and the graphics pipeline.
//
// Shaders are loaded as pre-compiled SPIR-V from shaderDir:
//   triangle.vert.spv  — hardcoded triangle, push-constant rotation angle
//   triangle.frag.spv  — interpolated vertex colour pass-through
class Pipeline {
public:
    Pipeline(const Context&              ctx,
             const Swapchain&            swapchain,
             const std::filesystem::path& shaderDir);
    ~Pipeline();

    Pipeline(const Pipeline&)            = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    vk::RenderPass     renderPass() const { return m_renderPass; }
    vk::PipelineLayout layout()     const { return m_layout;     }
    vk::Pipeline       handle()     const { return m_pipeline;   }

private:
    vk::ShaderModule loadShader(const std::filesystem::path& spvPath);

    const Context& m_ctx;
    vk::RenderPass     m_renderPass;
    vk::PipelineLayout m_layout;
    vk::Pipeline       m_pipeline;
};

} // namespace apeiron::render
