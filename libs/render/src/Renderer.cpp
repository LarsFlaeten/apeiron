#include "apeiron/render/Renderer.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Mesh.h"

#include <glm/glm.hpp>
#include <stdexcept>

namespace apeiron::render {

Renderer::Renderer(const Context&  ctx,
                   const Swapchain& swapchain,
                   const Pipeline&  pipeline)
    : m_ctx(ctx), m_swapchain(swapchain), m_pipeline(pipeline)
{
    auto device = ctx.device();

    // ----- Framebuffers -----
    for (auto view : swapchain.imageViews()) {
        std::array<vk::ImageView, 2> attachments{view, swapchain.depthImageView()};
        vk::FramebufferCreateInfo fbInfo{};
        fbInfo.setRenderPass(pipeline.renderPass())
              .setAttachments(attachments)
              .setWidth (swapchain.extent().width)
              .setHeight(swapchain.extent().height)
              .setLayers(1);
        m_framebuffers.push_back(device.createFramebuffer(fbInfo));
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

    // ----- Descriptor pool for body textures (capacity for up to 64 bodies) -----
    vk::DescriptorPoolSize poolSize{};
    poolSize.setType           (vk::DescriptorType::eCombinedImageSampler)
            .setDescriptorCount(64);
    vk::DescriptorPoolCreateInfo dpInfo{};
    dpInfo.setMaxSets  (64)
          .setPoolSizes(poolSize);
    m_descriptorPool = device.createDescriptorPool(dpInfo);

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

    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        device.destroySemaphore(m_imageAvailable[i]);
        device.destroySemaphore(m_renderFinished[i]);
        device.destroyFence    (m_inFlight[i]);
    }
    device.destroyDescriptorPool(m_descriptorPool);
    device.destroyCommandPool   (m_commandPool);
    for (auto fb : m_framebuffers)
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
    clearValues[0].color        = vk::ClearColorValue{std::array<float,4>{0.01f, 0.01f, 0.02f, 1.0f}};
    clearValues[1].depthStencil = vk::ClearDepthStencilValue{1.0f, 0};

    vk::RenderPassBeginInfo rpBegin{};
    rpBegin.setRenderPass (m_pipeline.renderPass())
           .setFramebuffer(m_framebuffers[m_activeImageIndex])
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

void Renderer::draw(const glm::mat4& mvp, const glm::mat4& model,
                    const glm::vec3& sunDir, vk::DescriptorSet descriptorSet,
                    const Mesh& mesh)
{
    // Push constant layout (128 bytes total, matches triangle.vert/.frag):
    //   offset  0: mat4 mvp          (64 bytes)
    //   offset 64: mat3 normalMat    (48 bytes — 3 columns, each padded to vec4)
    //   offset112: vec3 sunDir       (12 bytes + 4 pad)
    struct PushConstants {
        glm::mat4 mvp;
        glm::vec4 normalCol[3]; // mat3 columns, each padded to vec4
        glm::vec3 sunDir;
        float     _pad = 0.0f;
    };
    static_assert(sizeof(PushConstants) == 128);

    glm::mat3 nm = glm::mat3(model);
    PushConstants pc{};
    pc.mvp           = mvp;
    pc.normalCol[0]  = glm::vec4(nm[0], 0.0f);
    pc.normalCol[1]  = glm::vec4(nm[1], 0.0f);
    pc.normalCol[2]  = glm::vec4(nm[2], 0.0f);
    pc.sunDir        = sunDir;

    auto& cmd = m_commandBuffers[m_currentFrame];
    cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                           m_pipeline.layout(), 0, descriptorSet, {});
    cmd.pushConstants(m_pipeline.layout(),
                      vk::ShaderStageFlagBits::eVertex |
                      vk::ShaderStageFlagBits::eFragment,
                      0, sizeof(PushConstants), &pc);
    mesh.draw(cmd);
}

vk::DescriptorSet Renderer::allocateDescriptorSet(vk::ImageView view,
                                                   vk::Sampler   sampler)
{
    auto layout = m_pipeline.descriptorSetLayout();
    vk::DescriptorSetAllocateInfo allocInfo{};
    allocInfo.setDescriptorPool(m_descriptorPool)
             .setSetLayouts    (layout);
    auto sets = m_ctx.device().allocateDescriptorSets(allocInfo);

    vk::DescriptorImageInfo imgInfo{};
    imgInfo.setImageLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
           .setImageView  (view)
           .setSampler    (sampler);

    vk::WriteDescriptorSet write{};
    write.setDstSet         (sets[0])
         .setDstBinding     (0)
         .setDescriptorType (vk::DescriptorType::eCombinedImageSampler)
         .setImageInfo      (imgInfo);
    m_ctx.device().updateDescriptorSets(write, {});

    return sets[0];
}

void Renderer::endFrame()
{
    int  frame = m_currentFrame;
    auto& cmd  = m_commandBuffers[frame];

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

} // namespace apeiron::render
