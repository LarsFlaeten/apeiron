#pragma once

#include <vulkan/vulkan.hpp>
#include <filesystem>

namespace apeiron::render {

class Context;
class Swapchain;

// Owns the render pass, pipeline layout, and two graphics pipelines:
// one with filled polygons (default) and one with wireframe lines.
// Both share the same render pass, layout, and shaders.
class Pipeline {
public:
    Pipeline(const Context&               ctx,
             const Swapchain&             swapchain,
             const std::filesystem::path& shaderDir);
    ~Pipeline();

    Pipeline(const Pipeline&)            = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    vk::RenderPass          renderPass()           const { return m_renderPass;          }
    vk::PipelineLayout      layout()               const { return m_layout;               }
    vk::DescriptorSetLayout descriptorSetLayout()  const { return m_descriptorSetLayout;  }

    // Select the active pipeline variant.
    vk::Pipeline handle(bool wireframe = false) const
    {
        return wireframe ? m_wireframePipeline : m_fillPipeline;
    }

private:
    vk::ShaderModule loadShader(const std::filesystem::path& spvPath);
    vk::Pipeline     buildPipeline(vk::PolygonMode  polygonMode,
                                   vk::ShaderModule vert,
                                   vk::ShaderModule tesc,
                                   vk::ShaderModule tese,
                                   vk::ShaderModule frag);

    const Context&          m_ctx;
    vk::DescriptorSetLayout m_descriptorSetLayout;
    vk::RenderPass          m_renderPass;
    vk::PipelineLayout      m_layout;
    vk::Pipeline            m_fillPipeline;
    vk::Pipeline            m_wireframePipeline;
};

} // namespace apeiron::render
