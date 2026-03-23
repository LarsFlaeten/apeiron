#pragma once

#include <vulkan/vulkan.hpp>
#include <filesystem>

namespace apeiron::render {

class Context;
class Swapchain;

// A simple two-triangle (fullscreen) pipeline that reads the HDR offscreen
// image and writes ACES-tonemapped output directly to the swapchain.
// Push constant: float exposure (applied before tonemapping).
class TonemapPipeline {
public:
    TonemapPipeline(const Context&               ctx,
                    const Swapchain&             swapchain,
                    const std::filesystem::path& shaderDir);
    ~TonemapPipeline();

    TonemapPipeline(const TonemapPipeline&)            = delete;
    TonemapPipeline& operator=(const TonemapPipeline&) = delete;

    vk::RenderPass          renderPass()          const { return m_renderPass;          }
    vk::Pipeline            handle()              const { return m_pipeline;             }
    vk::PipelineLayout      layout()              const { return m_layout;               }
    vk::DescriptorSetLayout descriptorSetLayout() const { return m_descriptorSetLayout;  }

private:
    const Context& m_ctx;

    vk::RenderPass          m_renderPass;
    vk::DescriptorSetLayout m_descriptorSetLayout;
    vk::PipelineLayout      m_layout;
    vk::Pipeline            m_pipeline;
};

} // namespace apeiron::render
