#include "apeiron/render/Renderer.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/TonemapPipeline.h"
#include "apeiron/render/Mesh.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <stdexcept>

namespace apeiron::render {

Renderer::Renderer(const Context&         ctx,
                   const Swapchain&       swapchain,
                   const Pipeline&        pipeline,
                   const TonemapPipeline& tonemapPipeline)
    : m_ctx(ctx), m_swapchain(swapchain),
      m_pipeline(pipeline), m_tonemapPipeline(tonemapPipeline)
{
    auto device = ctx.device();

    // ----- HDR framebuffers (one per frame-in-flight) -----
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        std::array<vk::ImageView, 2> attachments{
            swapchain.hdrImageView(i),
            swapchain.depthImageView()
        };
        vk::FramebufferCreateInfo fbInfo{};
        fbInfo.setRenderPass(pipeline.renderPass())
              .setAttachments(attachments)
              .setWidth (swapchain.extent().width)
              .setHeight(swapchain.extent().height)
              .setLayers(1);
        m_hdrFramebuffers[i] = device.createFramebuffer(fbInfo);
    }

    // ----- Tonemap framebuffers (one per swapchain image) -----
    for (auto view : swapchain.imageViews()) {
        vk::FramebufferCreateInfo fbInfo{};
        fbInfo.setRenderPass(tonemapPipeline.renderPass())
              .setAttachments(view)
              .setWidth (swapchain.extent().width)
              .setHeight(swapchain.extent().height)
              .setLayers(1);
        m_tonemapFramebuffers.push_back(device.createFramebuffer(fbInfo));
    }

    // ----- Command pool -----
    vk::CommandPoolCreateInfo poolInfo{};
    poolInfo.setQueueFamilyIndex(ctx.graphicsFamily())
            .setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
    m_commandPool = device.createCommandPool(poolInfo);

    // ----- Command buffers (one per in-flight frame) -----
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setCommandPool       (m_commandPool)
             .setLevel             (vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(kMaxFramesInFlight);
    m_commandBuffers = device.allocateCommandBuffers(allocInfo);

    // ----- Material descriptor pool: 4 texture slots × 64 bodies -----
    vk::DescriptorPoolSize matPoolSize{};
    matPoolSize.setType           (vk::DescriptorType::eCombinedImageSampler)
               .setDescriptorCount(64 * 4);
    vk::DescriptorPoolCreateInfo dpInfo{};
    dpInfo.setMaxSets  (64)
          .setPoolSizes(matPoolSize);
    m_descriptorPool = device.createDescriptorPool(dpInfo);

    // ----- Tonemap descriptor pool + sampler + sets -----
    vk::SamplerCreateInfo samplerInfo{};
    samplerInfo.setMagFilter      (vk::Filter::eLinear)
               .setMinFilter      (vk::Filter::eLinear)
               .setAddressModeU   (vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeV   (vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeW   (vk::SamplerAddressMode::eClampToEdge);
    m_hdrSampler = device.createSampler(samplerInfo);

    vk::DescriptorPoolSize tmPoolSize{};
    tmPoolSize.setType           (vk::DescriptorType::eCombinedImageSampler)
              .setDescriptorCount(kMaxFramesInFlight);
    vk::DescriptorPoolCreateInfo tmPoolInfo{};
    tmPoolInfo.setMaxSets  (kMaxFramesInFlight)
              .setPoolSizes(tmPoolSize);
    m_tonemapDescPool = device.createDescriptorPool(tmPoolInfo);

    auto layout = tonemapPipeline.descriptorSetLayout();
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        vk::DescriptorSetAllocateInfo dsAlloc{};
        dsAlloc.setDescriptorPool(m_tonemapDescPool)
               .setSetLayouts    (layout);
        m_tonemapDescSets[i] = device.allocateDescriptorSets(dsAlloc)[0];

        vk::DescriptorImageInfo imgInfo{};
        imgInfo.setImageLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
               .setImageView  (swapchain.hdrImageView(i))
               .setSampler    (m_hdrSampler);
        vk::WriteDescriptorSet write{};
        write.setDstSet        (m_tonemapDescSets[i])
             .setDstBinding    (0)
             .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
             .setImageInfo     (imgInfo);
        device.updateDescriptorSets(write, {});
    }

    // ----- Per-frame sync objects -----
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        m_imageAvailable[i] = device.createSemaphore({});
        m_renderFinished[i] = device.createSemaphore({});
        m_inFlight[i]       = device.createFence({vk::FenceCreateFlagBits::eSignaled});
    }
}

Renderer::~Renderer()
{
    auto device = m_ctx.device();
    device.waitIdle();

    if (m_imguiPool) {
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        device.destroyDescriptorPool(m_imguiPool);
    }

    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        device.destroySemaphore(m_imageAvailable[i]);
        device.destroySemaphore(m_renderFinished[i]);
        device.destroyFence    (m_inFlight[i]);
    }

    device.destroySampler       (m_hdrSampler);
    device.destroyDescriptorPool(m_tonemapDescPool);
    device.destroyDescriptorPool(m_descriptorPool);
    device.destroyCommandPool   (m_commandPool);

    for (auto fb : m_tonemapFramebuffers)
        device.destroyFramebuffer(fb);
    for (auto fb : m_hdrFramebuffers)
        device.destroyFramebuffer(fb);
}

bool Renderer::beginFrame()
{
    auto device = m_ctx.device();
    int  frame  = m_currentFrame;

    (void)device.waitForFences(m_inFlight[frame], vk::True, UINT64_MAX);

    auto [acqResult, imageIndex] = device.acquireNextImageKHR(
        m_swapchain.handle(), UINT64_MAX, m_imageAvailable[frame], nullptr);

    if (acqResult == vk::Result::eErrorOutOfDateKHR)
        return false;

    device.resetFences(m_inFlight[frame]);
    m_activeImageIndex = imageIndex;

    auto& cmd = m_commandBuffers[frame];
    cmd.reset();
    cmd.begin(vk::CommandBufferBeginInfo{});

    std::array<vk::ClearValue, 2> clearValues{};
    clearValues[0].color        = vk::ClearColorValue{std::array<float,4>{0.0f, 0.0f, 0.0f, 1.0f}};
    clearValues[1].depthStencil = vk::ClearDepthStencilValue{1.0f, 0};

    vk::RenderPassBeginInfo rpBegin{};
    rpBegin.setRenderPass (m_pipeline.renderPass())
           .setFramebuffer(m_hdrFramebuffers[frame])
           .setRenderArea ({vk::Offset2D{0, 0}, m_swapchain.extent()})
           .setClearValues(clearValues);

    cmd.beginRenderPass(rpBegin, vk::SubpassContents::eInline);
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, m_pipeline.handle(m_wireframe));

    vk::Viewport vp{
        0.0f, 0.0f,
        static_cast<float>(m_swapchain.extent().width),
        static_cast<float>(m_swapchain.extent().height),
        0.0f, 1.0f
    };
    cmd.setViewport(0, vp);
    cmd.setScissor (0, vk::Rect2D{{0, 0}, m_swapchain.extent()});

    return true;
}

void Renderer::draw(const glm::mat4&  mvp,
                    const glm::mat4&  model,
                    const glm::vec3&  sunDir,
                    const glm::vec3&  viewDir,
                    bool              isEmissive,
                    float             lightIntensity,
                    vk::DescriptorSet descriptorSet,
                    const Mesh&       mesh)
{
    // Push constant layout (128 bytes):
    //   offset  0: mat4 mvp            (64 bytes)
    //   offset 64: vec4 normalCol[2]   (32 bytes)
    //   offset 96: vec4 sunDirPad      (xyz=sunDir, w=isEmissive flag)
    //  offset112: vec4 viewDirPad     (xyz=viewDir, w=lightIntensity)
    struct PushConstants {
        glm::mat4 mvp;
        glm::vec4 normalCol[2];
        glm::vec4 sunDirPad;
        glm::vec4 viewDirPad;
    };
    static_assert(sizeof(PushConstants) == 128);

    glm::mat3 nm = glm::mat3(model);
    PushConstants pc{};
    pc.mvp          = mvp;
    pc.normalCol[0] = glm::vec4(nm[0], 0.0f);
    pc.normalCol[1] = glm::vec4(nm[1], 0.0f);
    pc.sunDirPad    = glm::vec4(sunDir,  isEmissive ? 1.0f : 0.0f);
    pc.viewDirPad   = glm::vec4(viewDir, lightIntensity);

    auto& cmd = m_commandBuffers[m_currentFrame];
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, m_pipeline.handle(m_wireframe));
    cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                           m_pipeline.layout(), 0, descriptorSet, {});
    cmd.pushConstants(m_pipeline.layout(),
                      vk::ShaderStageFlagBits::eVertex |
                      vk::ShaderStageFlagBits::eFragment,
                      0, sizeof(PushConstants), &pc);
    mesh.draw(cmd);
}

vk::DescriptorSet Renderer::allocateDescriptorSet(
    const std::array<vk::ImageView, 4>& views,
    const std::array<vk::Sampler,   4>& samplers)
{
    auto layout = m_pipeline.descriptorSetLayout();
    vk::DescriptorSetAllocateInfo allocInfo{};
    allocInfo.setDescriptorPool(m_descriptorPool)
             .setSetLayouts    (layout);
    auto sets = m_ctx.device().allocateDescriptorSets(allocInfo);

    std::array<vk::DescriptorImageInfo, 4> imgInfos{};
    std::array<vk::WriteDescriptorSet,  4> writes{};
    for (uint32_t i = 0; i < 4; ++i) {
        imgInfos[i].setImageLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
                   .setImageView  (views[i])
                   .setSampler    (samplers[i]);
        writes[i].setDstSet        (sets[0])
                 .setDstBinding    (i)
                 .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                 .setImageInfo     (imgInfos[i]);
    }
    m_ctx.device().updateDescriptorSets(writes, {});
    return sets[0];
}

void Renderer::endFrame()
{
    int   frame = m_currentFrame;
    auto& cmd   = m_commandBuffers[frame];

    // ---- End HDR render pass ----
    cmd.endRenderPass();

    // ---- Tonemap pass ----
    // The HDR render pass finalLayout + outbound subpass dependency already
    // transitioned the image to eShaderReadOnlyOptimal and ensured visibility,
    // so no explicit barrier is needed here.
    vk::RenderPassBeginInfo tmBegin{};
    tmBegin.setRenderPass (m_tonemapPipeline.renderPass())
           .setFramebuffer(m_tonemapFramebuffers[m_activeImageIndex])
           .setRenderArea ({vk::Offset2D{0, 0}, m_swapchain.extent()});

    cmd.beginRenderPass(tmBegin, vk::SubpassContents::eInline);
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, m_tonemapPipeline.handle());
    cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                           m_tonemapPipeline.layout(), 0,
                           m_tonemapDescSets[frame], {});
    cmd.pushConstants(m_tonemapPipeline.layout(),
                      vk::ShaderStageFlagBits::eFragment,
                      0, sizeof(float), &m_exposure);
    cmd.draw(3, 1, 0, 0);  // fullscreen triangle, no vertex buffer

    // ImGui renders in the tonemap pass so the UI bypasses tonemapping.
    if (m_imguiPool)
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(),
            static_cast<VkCommandBuffer>(cmd));

    cmd.endRenderPass();
    cmd.end();

    vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    vk::SubmitInfo submitInfo{};
    submitInfo.setWaitSemaphores  (m_imageAvailable[frame])
              .setWaitDstStageMask(waitStage)
              .setCommandBuffers  (cmd)
              .setSignalSemaphores(m_renderFinished[frame]);
    m_ctx.graphicsQueue().submit(submitInfo, m_inFlight[frame]);

    vk::SwapchainKHR   swapchain = m_swapchain.handle();
    vk::PresentInfoKHR presentInfo{};
    presentInfo.setWaitSemaphores(m_renderFinished[frame])
               .setSwapchains   (swapchain)
               .setImageIndices (m_activeImageIndex);
    (void)m_ctx.presentQueue().presentKHR(presentInfo);

    m_currentFrame = (m_currentFrame + 1) % kMaxFramesInFlight;
}

void Renderer::initImGui(GLFWwindow* window)
{
    auto device = m_ctx.device();

    vk::DescriptorPoolSize poolSize{};
    poolSize.setType(vk::DescriptorType::eCombinedImageSampler)
            .setDescriptorCount(16);
    vk::DescriptorPoolCreateInfo poolInfo{};
    poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet)
            .setMaxSets(16)
            .setPoolSizes(poolSize);
    m_imguiPool = device.createDescriptorPool(poolInfo);

    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForVulkan(window, /*install_callbacks=*/true);

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.Instance       = static_cast<VkInstance>       (m_ctx.instance());
    initInfo.PhysicalDevice = static_cast<VkPhysicalDevice> (m_ctx.physicalDevice());
    initInfo.Device         = static_cast<VkDevice>         (m_ctx.device());
    initInfo.QueueFamily    = m_ctx.graphicsFamily();
    initInfo.Queue          = static_cast<VkQueue>          (m_ctx.graphicsQueue());
    initInfo.DescriptorPool = static_cast<VkDescriptorPool> (m_imguiPool);
    initInfo.MinImageCount  = 2;
    initInfo.ImageCount     = m_swapchain.imageCount();
    // ImGui renders in the tonemap pass — use that render pass here.
    initInfo.PipelineInfoMain.RenderPass  = static_cast<VkRenderPass>(m_tonemapPipeline.renderPass());
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    ImGui_ImplVulkan_Init(&initInfo);
}

} // namespace apeiron::render
