#pragma once

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>

#include <array>
#include <vector>

namespace apeiron::render {

class Context;
class Swapchain;
class Pipeline;
class Mesh;

// Owns framebuffers, command pool/buffers, and per-frame sync objects.
// Drives the acquire → record → submit → present loop.
//
// Usage per frame:
//   if (!renderer.beginFrame()) continue;   // acquire + begin render pass
//   renderer.draw(mvp, model, meshA);       // any number of draw calls
//   renderer.draw(mvp, model, meshB);
//   renderer.endFrame();                    // submit + present
class Renderer {
public:
    Renderer(const Context&   ctx,
             const Swapchain& swapchain,
             const Pipeline&  pipeline);
    ~Renderer();

    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Acquire the next swapchain image and begin the render pass.
    // Returns false if the swapchain is out of date (resize) — skip the frame.
    bool beginFrame();

    // Record one mesh draw.  sunDir is the normalised world-space direction
    // toward the sun for this body (computed by the caller per body).
    // descriptorSet must have been allocated via allocateDescriptorSet().
    // Must be called between beginFrame() and endFrame().
    void draw(const glm::mat4&   mvp,
              const glm::mat4&   model,
              const glm::vec3&   sunDir,
              vk::DescriptorSet  descriptorSet,
              const Mesh&        mesh);

    // Allocate and write a descriptor set for a texture.  The returned set is
    // owned by the Renderer's pool and freed when the Renderer is destroyed.
    vk::DescriptorSet allocateDescriptorSet(vk::ImageView view,
                                            vk::Sampler   sampler);

    // End the render pass, submit, and present.
    void endFrame();

    void setWireframe(bool enabled) { m_wireframe = enabled; }
    bool wireframe()          const { return m_wireframe; }

private:
    static constexpr int kMaxFramesInFlight = 2;

    const Context&   m_ctx;
    const Swapchain& m_swapchain;
    const Pipeline&  m_pipeline;

    vk::DescriptorPool             m_descriptorPool;
    vk::CommandPool                m_commandPool;
    std::vector<vk::CommandBuffer> m_commandBuffers;
    std::vector<vk::Framebuffer>   m_framebuffers;

    std::array<vk::Semaphore, kMaxFramesInFlight> m_imageAvailable;
    std::array<vk::Semaphore, kMaxFramesInFlight> m_renderFinished;
    std::array<vk::Fence,     kMaxFramesInFlight> m_inFlight;

    int      m_currentFrame     = 0;
    uint32_t m_activeImageIndex = 0;
    bool     m_wireframe        = false;
};

} // namespace apeiron::render
