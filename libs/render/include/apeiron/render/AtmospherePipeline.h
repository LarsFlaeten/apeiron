#pragma once

#include <vulkan/vulkan.hpp>
#include <filesystem>

namespace apeiron::render {

class Context;
class Swapchain;

// Graphics pipeline for rendering the atmosphere shell.
//
// Separate from Pipeline because it has a different descriptor set layout
// (one transmittance LUT sampler instead of five material textures) and
// different push constants.
//
// Blend equation: src=One, dst=SrcAlpha
//   result = inscattered_light + background × transmittance
//
// Descriptor set layout (set 0):
//   binding 0 — combined image sampler 2D  (transmittance LUT)
//
// Push constant layout (128 bytes, stages Vertex | Fragment):
//   offset   0 — mat4  mvp                (64 bytes)
//   offset  64 — vec4  cameraLocalGr      (16 bytes)
//   offset  80 — vec4  sunDirLight        (16 bytes)
//   offset  96 — vec4  rayleighBeta       (16 bytes)
//   offset 112 — vec4  miePack            (16 bytes)
class AtmospherePipeline {
public:
    // renderPass must be the HDR render pass from Pipeline.
    AtmospherePipeline(const Context&               ctx,
                       vk::RenderPass               renderPass,
                       const std::filesystem::path& shaderDir);
    ~AtmospherePipeline();

    AtmospherePipeline(const AtmospherePipeline&)            = delete;
    AtmospherePipeline& operator=(const AtmospherePipeline&) = delete;

    vk::Pipeline            handle()            const { return m_pipeline;       }
    vk::PipelineLayout      layout()            const { return m_layout;         }
    vk::DescriptorSetLayout descriptorSetLayout() const { return m_descSetLayout; }

private:
    const Context&          m_ctx;
    vk::DescriptorSetLayout m_descSetLayout;
    vk::PipelineLayout      m_layout;
    vk::Pipeline            m_pipeline;
};

} // namespace apeiron::render
