#pragma once

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>
#include <filesystem>

namespace apeiron::render {

class Context;
class Swapchain;
class Pipeline;   // for the render pass + HDR framebuffers

// Push-constant block sent to mesh.vert / mesh.frag.
// Total = 2×mat4 + 2×vec4 = 160 bytes (well within the 128-byte minimum guarantee
// and within the 256-byte limit guaranteed by Vulkan spec for PC blocks).
struct MeshPushConstants {
    glm::mat4 mvp;
    glm::mat4 modelMat;
    glm::vec4 sunDir;    // xyz = direction toward sun, w = isEmissive (1=emissive)
    glm::vec4 baseColor; // xyz = colour tint, w = emissive intensity
};
static_assert(sizeof(MeshPushConstants) == 160);

// Lightweight non-tessellated graphics pipeline for rigid meshes (spacecraft, etc.).
// Shares the HDR render pass owned by Pipeline so it composites into the same
// HDR framebuffer alongside planets.
class MeshPipeline {
public:
    MeshPipeline(const Context&               ctx,
                 const Pipeline&              mainPipeline,  // for render pass
                 const std::filesystem::path& shaderDir);
    ~MeshPipeline();

    MeshPipeline(const MeshPipeline&)            = delete;
    MeshPipeline& operator=(const MeshPipeline&) = delete;

    // handle(false) = back-face culled (solid geometry)
    // handle(true)  = no culling (double-sided: flat panels)
    vk::Pipeline       handle(bool doubleSided = false) const {
        return doubleSided ? m_pipelineDS : m_pipeline;
    }
    // Dedicated plume pipeline: additive blend, no culling, plume shaders.
    vk::Pipeline       plumeHandle() const { return m_pipelinePlume; }
    vk::PipelineLayout layout()     const { return m_layout;   }
    vk::RenderPass     renderPass() const { return m_renderPass; }

private:
    vk::ShaderModule loadShader(const std::filesystem::path& spvPath);

    const Context&   m_ctx;
    vk::RenderPass   m_renderPass;   // borrowed from mainPipeline (not owned)
    vk::PipelineLayout m_layout;
    vk::Pipeline       m_pipeline;      // back-face culled
    vk::Pipeline       m_pipelineDS;   // double-sided (no culling)
    vk::Pipeline       m_pipelinePlume; // plume: additive blend, no cull, plume shaders
};

} // namespace apeiron::render
