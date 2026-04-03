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
    glm::vec4 baseColor; // xyz = colour tint, w = emissive intensity / plume intensity
};
static_assert(sizeof(MeshPushConstants) == 160);

// UBO layout for per-material descriptor set (set 0).
// Binding 0 = this UBO, binding 1 = albedo sampler2D.
struct MaterialUBO {
    glm::vec4 baseColor;    // RGB + unused alpha
    float     metallic;
    float     roughness;
    float     _pad0;
    float     _pad1;
};

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
    vk::Pipeline       plumeHandle()   const { return m_pipelinePlume; }
    vk::PipelineLayout layout()        const { return m_layout;        }
    vk::RenderPass     renderPass()    const { return m_renderPass;    }

    // Material descriptor set infrastructure — used by GltfModel at load time.
    // Set 0, binding 0: MaterialUBO.  Set 0, binding 1: albedo sampler2D.
    vk::DescriptorSetLayout matDescSetLayout() const { return m_matDescSetLayout; }
    vk::DescriptorPool      matDescPool()      const { return m_matDescPool;      }

    // Allocate one descriptor set from the material pool and write the given
    // buffer + imageView/sampler into it.  Caller owns the returned set
    // (it lives until the pool is reset or destroyed).
    vk::DescriptorSet allocateMaterialDescSet(vk::Buffer     ubo,
                                              vk::DeviceSize uboSize,
                                              vk::ImageView  imageView,
                                              vk::Sampler    sampler) const;

private:
    vk::ShaderModule loadShader(const std::filesystem::path& spvPath);
    void             createMaterialDescLayout();

    const Context&   m_ctx;
    vk::RenderPass   m_renderPass;      // borrowed from mainPipeline (not owned)
    vk::PipelineLayout m_layout;
    vk::Pipeline       m_pipeline;      // back-face culled
    vk::Pipeline       m_pipelineDS;    // double-sided (no culling)
    vk::Pipeline       m_pipelinePlume; // plume: additive blend, no cull, plume shaders

    vk::DescriptorSetLayout m_matDescSetLayout;
    vk::DescriptorPool      m_matDescPool;      // pool for material descriptor sets
};

} // namespace apeiron::render
