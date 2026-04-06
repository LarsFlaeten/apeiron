#pragma once

#include "apeiron/render/Image.h"

#include <vulkan/vulkan.hpp>
#include <imgui.h>

#include <array>

namespace apeiron::render {
class Context;
class GpuAllocator;
class Pipeline;
class Swapchain;
} // namespace apeiron::render

// ---------------------------------------------------------------------------
// OffscreenCam
//
// 512×512 RGBA16F offscreen render target for spacecraft camera views.
// Reuses Pipeline::renderPass() directly, so GltfModel::draw() via
// MeshPipeline is fully compatible — no additional pipelines needed.
//
// Lifecycle:
//   Declare BEFORE Renderer so it is destroyed AFTER Renderer::~Renderer()
//   (which calls device.waitIdle()), ensuring the GPU is idle first.
//   Call init() after renderer.initImGui() (needed for AddTexture).
//
// Per-frame usage (call BEFORE renderer.beginHDRPass()):
//   offscreenCam.begin(renderer.currentCmd(), renderer.currentFrameIndex());
//   gltfA.draw(renderer.currentCmd(), meshPipeline, vp, model, sun, camPos);
//   offscreenCam.end(renderer.currentCmd());
//   mfd.setTexture(offscreenCam.imguiTexture(renderer.currentFrameIndex()));
// ---------------------------------------------------------------------------
class OffscreenCam {
public:
    static constexpr uint32_t kWidth  = 512;
    static constexpr uint32_t kHeight = 512;

    OffscreenCam() = default;
    ~OffscreenCam();

    OffscreenCam(const OffscreenCam&)            = delete;
    OffscreenCam& operator=(const OffscreenCam&) = delete;

    // Initialise GPU resources.  Must be called after renderer.initImGui().
    void init(const apeiron::render::Context&   ctx,
              apeiron::render::GpuAllocator&    allocator,
              const apeiron::render::Pipeline&  pipeline,
              const apeiron::render::Swapchain& swapchain);

    // Release all GPU resources.  Called automatically by the destructor.
    void destroy();

    // Begin the offscreen render pass for this frame.
    void begin(vk::CommandBuffer cmd, int frameIndex);

    // End the render pass.  The color attachment transitions to
    // eShaderReadOnlyOptimal, ready for ImGui to sample.
    void end(vk::CommandBuffer cmd);

    // ImTextureID for the given frame's color output.
    ImTextureID imguiTexture(int frameIndex) const;

private:
    static constexpr int kFrames = 2;  // must match Renderer::kMaxFramesInFlight

    vk::Device     m_device{};
    vk::RenderPass m_renderPass{};  // borrowed from Pipeline — not owned
    vk::Sampler    m_sampler{};

    // Single depth image shared between frames (sequential rendering only).
    apeiron::render::Image m_depthImage;
    vk::ImageView          m_depthView{};

    // Per-frame color images to avoid R/W hazards with kMaxFramesInFlight > 1.
    std::array<apeiron::render::Image, kFrames> m_colorImages;
    std::array<vk::ImageView,          kFrames> m_colorViews{};
    std::array<vk::Framebuffer,        kFrames> m_framebuffers{};

    // ImGui descriptor sets (one per frame).  Freed implicitly when the
    // ImGui descriptor pool is destroyed inside Renderer::~Renderer().
    std::array<VkDescriptorSet, kFrames> m_imguiSets{};
};
