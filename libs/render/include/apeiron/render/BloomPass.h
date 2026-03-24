#pragma once

#include "apeiron/render/Image.h"

#include <vulkan/vulkan.hpp>
#include <filesystem>
#include <array>

namespace apeiron::render {

class Context;
class GpuAllocator;
class Swapchain;

// Threshold + separable-Gaussian-blur bloom post-process.
// Operates at half the swapchain resolution.
// Two full sets of ping-pong images (one per frame-in-flight) prevent
// write/read hazards when kMaxFramesInFlight > 1.
//
// Usage per frame:
//   bloomPass.render(cmd, currentFrame);        // after HDR pass ends
//   // tonemap pass reads outputImageView(currentFrame) at binding 1
class BloomPass {
public:
    BloomPass(const Context&,
              GpuAllocator&,
              const Swapchain&,
              const std::filesystem::path& shaderDir);
    ~BloomPass();

    BloomPass(const BloomPass&)            = delete;
    BloomPass& operator=(const BloomPass&) = delete;

    // Execute threshold + 2× H/V Gaussian blur.
    // Call after the HDR render pass has ended.
    void render(vk::CommandBuffer cmd, int frameIndex);

    // Per-frame bloom output (for tonemap descriptor binding 1).
    vk::ImageView outputImageView(int frameIndex) const { return m_bloomViews[frameIndex][0]; }
    vk::Sampler   sampler()                       const { return m_sampler; }

    // Rebuild size-dependent resources after Swapchain::recreate().
    // Also updates threshold descriptor sets with the new HDR image views.
    void recreate(const Swapchain&);

    float threshold() const { return m_threshold; }
    float strength()  const { return m_strength;  }
    void  setThreshold(float t) { m_threshold = t; }
    void  setStrength (float s) { m_strength  = s; }

private:
    void createImages      (const Swapchain&);
    void destroyImages     ();
    void createFramebuffers();
    void destroyFramebuffers();
    void updateDescriptors (const Swapchain&);

    const Context& m_ctx;
    GpuAllocator*  m_allocator;

    vk::RenderPass          m_renderPass;
    vk::DescriptorSetLayout m_descSetLayout;
    vk::PipelineLayout      m_pipelineLayout;
    vk::Pipeline            m_thresholdPipeline;
    vk::Pipeline            m_blurPipeline;
    vk::DescriptorPool      m_descPool;
    vk::Sampler             m_sampler;

    // m_sets[0,1]  : threshold — reads HDR image 0 or 1
    // m_sets[2,3]  : blur for frame 0 — reads bloom[0][0] or bloom[0][1]
    // m_sets[4,5]  : blur for frame 1 — reads bloom[1][0] or bloom[1][1]
    std::array<vk::DescriptorSet, 6> m_sets{};

    // [frame-in-flight][ping-pong]
    std::array<std::array<Image,           2>, 2> m_bloomImages;
    std::array<std::array<vk::ImageView,   2>, 2> m_bloomViews{};
    std::array<std::array<vk::Framebuffer, 2>, 2> m_framebuffers{};

    vk::Extent2D m_extent{};
    float        m_threshold = 0.8f;
    float        m_strength  = 0.3f;
};

} // namespace apeiron::render
