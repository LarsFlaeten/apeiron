#pragma once

#include <vulkan/vulkan.hpp>

#include <array>
#include <vector>

namespace apeiron::render {

class Context;
class Swapchain;
class Pipeline;

// Owns framebuffers, command pool/buffers, and per-frame sync objects.
// Drives the acquire → record → submit → present loop.
class Renderer {
public:
    Renderer(const Context&  ctx,
             const Swapchain& swapchain,
             const Pipeline&  pipeline);
    ~Renderer();

    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Draw one frame.  angle (radians) is pushed to the vertex shader.
    void drawFrame(float angle);

private:
    static constexpr int kMaxFramesInFlight = 2;

    const Context&   m_ctx;
    const Swapchain& m_swapchain;
    const Pipeline&  m_pipeline;

    vk::CommandPool                m_commandPool;
    std::vector<vk::CommandBuffer> m_commandBuffers;
    std::vector<vk::Framebuffer>   m_framebuffers;

    std::array<vk::Semaphore, kMaxFramesInFlight> m_imageAvailable;
    std::array<vk::Semaphore, kMaxFramesInFlight> m_renderFinished;
    std::array<vk::Fence,     kMaxFramesInFlight> m_inFlight;

    int m_currentFrame = 0;
};

} // namespace apeiron::render
