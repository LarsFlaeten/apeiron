#pragma once

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>

#include <array>
#include <vector>

struct GLFWwindow;

namespace apeiron::render {

class Context;
class Swapchain;
class Pipeline;
class TonemapPipeline;
class Mesh;

// Owns framebuffers, command pool/buffers, and per-frame sync objects.
// Drives the acquire → record → submit → present loop.
//
// Two-pass frame:
//   1. HDR pass  — main geometry drawn to an RGBA16F offscreen image.
//   2. Tonemap pass — fullscreen triangle applies ACES + exposure and
//      writes to the swapchain image.  ImGui renders here so the UI
//      is not tonemapped.
//
// Usage per frame:
//   if (!renderer.beginFrame()) continue;
//   renderer.draw(mvp, model, ...);
//   renderer.endFrame();           // runs both passes, then presents
class Renderer {
public:
    static constexpr int kMaxFramesInFlight = 2;

    Renderer(const Context&        ctx,
             const Swapchain&      swapchain,
             const Pipeline&       pipeline,
             const TonemapPipeline& tonemapPipeline);
    ~Renderer();

    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Acquire the next swapchain image and begin the HDR render pass.
    // Returns false if the swapchain is out of date (resize) — skip the frame.
    bool beginFrame();

    // Record one mesh draw.
    // sunDir/viewDir are normalised world-space vectors (computed by caller).
    // isEmissive = true for self-luminous bodies (the sun); skips lighting.
    // lightIntensity = 1/d² in AU for physically-based brightness falloff.
    // descriptorSet must have been allocated via allocateDescriptorSet().
    void draw(const glm::mat4&  mvp,
              const glm::mat4&  model,
              const glm::vec3&  sunDir,
              const glm::vec3&  viewDir,
              bool              isEmissive,
              float             lightIntensity,
              vk::DescriptorSet descriptorSet,
              const Mesh&       mesh);

    // Allocate and write a descriptor set for 4 material texture slots:
    // [0]=diffuse  [1]=specular  [2]=normals  [3]=clouds
    vk::DescriptorSet allocateDescriptorSet(
        const std::array<vk::ImageView, 4>& views,
        const std::array<vk::Sampler,   4>& samplers);

    // End both passes, submit, and present.
    void endFrame();

    // Initialise Dear ImGui (call once after construction).
    void initImGui(GLFWwindow* window);

    void setWireframe(bool enabled) { m_wireframe = enabled; }
    bool wireframe()          const { return m_wireframe; }

    void  setExposure(float e) { m_exposure = e; }
    float exposure()     const { return m_exposure; }

private:
    const Context&         m_ctx;
    const Swapchain&       m_swapchain;
    const Pipeline&        m_pipeline;
    const TonemapPipeline& m_tonemapPipeline;

    // Material descriptor pool (bodies).
    vk::DescriptorPool             m_descriptorPool;
    // Tonemap descriptor pool + per-frame sets (HDR sampler).
    vk::DescriptorPool             m_tonemapDescPool;
    std::array<vk::DescriptorSet,  kMaxFramesInFlight> m_tonemapDescSets;
    vk::Sampler                    m_hdrSampler;

    vk::DescriptorPool             m_imguiPool;
    vk::CommandPool                m_commandPool;
    std::vector<vk::CommandBuffer> m_commandBuffers;

    // Main (HDR) framebuffers — one per frame-in-flight.
    std::array<vk::Framebuffer, kMaxFramesInFlight> m_hdrFramebuffers;
    // Tonemap framebuffers — one per swapchain image.
    std::vector<vk::Framebuffer>                    m_tonemapFramebuffers;

    std::array<vk::Semaphore, kMaxFramesInFlight> m_imageAvailable;
    std::array<vk::Semaphore, kMaxFramesInFlight> m_renderFinished;
    std::array<vk::Fence,     kMaxFramesInFlight> m_inFlight;

    int      m_currentFrame     = 0;
    uint32_t m_activeImageIndex = 0;
    bool     m_wireframe        = false;
    float    m_exposure         = 1.0f;
};

} // namespace apeiron::render
