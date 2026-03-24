#pragma once

#include "apeiron/render/Buffer.h"

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>
#include <filesystem>
#include <vector>

namespace apeiron::render {

class Context;
class GpuAllocator;
class Swapchain;

struct StarVertex {
    glm::vec3 direction;  // unit vector in render frame (ECLIPJ2000)
    glm::vec3 color;      // linear RGB already multiplied by HDR brightness
};

// Renders a static point-cloud of background stars.
// Must be drawn first in the HDR render pass (depth test and write are off).
// Push constant: mat4 vpRot — the combined VP matrix with the translation
// column zeroed so stars appear at infinity regardless of camera position.
class StarField {
public:
    StarField(const Context&               ctx,
              GpuAllocator&                allocator,
              vk::RenderPass               hdrRenderPass,
              const Swapchain&             swapchain,
              const std::filesystem::path& shaderDir,
              std::vector<StarVertex>       stars);
    ~StarField();

    StarField(const StarField&)            = delete;
    StarField& operator=(const StarField&) = delete;

    void draw(vk::CommandBuffer cmd, const glm::mat4& vpRot) const;

private:
    const Context& m_ctx;

    vk::PipelineLayout m_layout;
    vk::Pipeline       m_pipeline;

    Buffer   m_vertexBuffer;
    uint32_t m_starCount = 0;
};

} // namespace apeiron::render
