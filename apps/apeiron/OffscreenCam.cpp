#include "OffscreenCam.h"

#include "apeiron/render/Context.h"
#include "apeiron/render/GpuAllocator.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Swapchain.h"

#include <imgui_impl_vulkan.h>

#include <array>

// ---------------------------------------------------------------------------
OffscreenCam::~OffscreenCam()
{
    if (m_device)
        destroy();
}

// ---------------------------------------------------------------------------
void OffscreenCam::init(
    const apeiron::render::Context&   ctx,
    apeiron::render::GpuAllocator&    allocator,
    const apeiron::render::Pipeline&  pipeline,
    const apeiron::render::Swapchain& swapchain)
{
    m_device     = ctx.device();
    m_renderPass = pipeline.renderPass();  // borrow — pipeline owns this

    // ---- Sampler (linear, clamp-to-edge) ----
    vk::SamplerCreateInfo samplerInfo{};
    samplerInfo.setMagFilter   (vk::Filter::eLinear)
               .setMinFilter   (vk::Filter::eLinear)
               .setAddressModeU(vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeV(vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeW(vk::SamplerAddressMode::eClampToEdge);
    m_sampler = m_device.createSampler(samplerInfo);

    // ---- Shared depth image ----
    // initialLayout = eUndefined in the render pass → cleared every frame; sharing is safe.
    m_depthImage = apeiron::render::Image(
        allocator.handle(),
        {kWidth, kHeight},
        swapchain.depthFormat(),
        vk::ImageUsageFlagBits::eDepthStencilAttachment);

    vk::ImageViewCreateInfo dvInfo{};
    dvInfo.setImage           (m_depthImage.handle())
          .setViewType        (vk::ImageViewType::e2D)
          .setFormat          (swapchain.depthFormat())
          .setSubresourceRange({vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1});
    m_depthView = m_device.createImageView(dvInfo);

    // ---- Per-frame color images ----
    for (int i = 0; i < kFrames; ++i) {
        m_colorImages[i] = apeiron::render::Image(
            allocator.handle(),
            {kWidth, kHeight},
            vk::Format::eR16G16B16A16Sfloat,
            vk::ImageUsageFlagBits::eColorAttachment |
            vk::ImageUsageFlagBits::eSampled);

        vk::ImageViewCreateInfo cvInfo{};
        cvInfo.setImage           (m_colorImages[i].handle())
              .setViewType        (vk::ImageViewType::e2D)
              .setFormat          (vk::Format::eR16G16B16A16Sfloat)
              .setSubresourceRange({vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1});
        m_colorViews[i] = m_device.createImageView(cvInfo);

        // ---- Framebuffer (uses the HDR render pass → compatible with MeshPipeline) ----
        std::array<vk::ImageView, 2> attachments{ m_colorViews[i], m_depthView };
        vk::FramebufferCreateInfo fbInfo{};
        fbInfo.setRenderPass(m_renderPass)
              .setAttachments (attachments)
              .setWidth       (kWidth)
              .setHeight      (kHeight)
              .setLayers      (1);
        m_framebuffers[i] = m_device.createFramebuffer(fbInfo);

        // ---- Register with ImGui ----
        // The render pass final layout is eShaderReadOnlyOptimal, which matches here.
        m_imguiSets[i] = ImGui_ImplVulkan_AddTexture(
            static_cast<VkSampler>   (m_sampler),
            static_cast<VkImageView> (m_colorViews[i]),
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
}

// ---------------------------------------------------------------------------
void OffscreenCam::destroy()
{
    if (!m_device) return;

    // ImGui descriptor sets are freed implicitly when the ImGui pool is
    // destroyed inside Renderer::~Renderer(), which runs before this
    // destructor (Renderer is declared after OffscreenCam in main.cpp).
    m_imguiSets.fill(VK_NULL_HANDLE);

    for (int i = 0; i < kFrames; ++i) {
        if (m_framebuffers[i]) m_device.destroyFramebuffer(m_framebuffers[i]);
        if (m_colorViews[i])   m_device.destroyImageView  (m_colorViews[i]);
        m_colorImages[i]  = apeiron::render::Image{};
        m_framebuffers[i] = nullptr;
        m_colorViews[i]   = nullptr;
    }
    if (m_depthView) m_device.destroyImageView(m_depthView);
    m_depthImage = apeiron::render::Image{};
    m_depthView  = nullptr;

    if (m_sampler) m_device.destroySampler(m_sampler);
    m_sampler = nullptr;
    m_device  = nullptr;
}

// ---------------------------------------------------------------------------
void OffscreenCam::begin(vk::CommandBuffer cmd, int frameIndex)
{
    std::array<vk::ClearValue, 2> clears{};
    clears[0].color        = vk::ClearColorValue{std::array<float, 4>{0.0f, 0.0f, 0.0f, 1.0f}};
    clears[1].depthStencil = vk::ClearDepthStencilValue{1.0f, 0};

    vk::RenderPassBeginInfo rpBegin{};
    rpBegin.setRenderPass (m_renderPass)
           .setFramebuffer(m_framebuffers[frameIndex])
           .setRenderArea ({vk::Offset2D{0, 0}, {kWidth, kHeight}})
           .setClearValues(clears);

    cmd.beginRenderPass(rpBegin, vk::SubpassContents::eInline);

    vk::Viewport vp{0.0f, 0.0f,
                    static_cast<float>(kWidth), static_cast<float>(kHeight),
                    0.0f, 1.0f};
    cmd.setViewport(0, vp);
    cmd.setScissor (0, vk::Rect2D{{0, 0}, {kWidth, kHeight}});
}

// ---------------------------------------------------------------------------
void OffscreenCam::end(vk::CommandBuffer cmd)
{
    cmd.endRenderPass();
    // The render pass final layout transition moves the color attachment to
    // eShaderReadOnlyOptimal; the outbound subpass dependency ensures it is
    // visible to fragment shaders before ImGui reads it.
}

// ---------------------------------------------------------------------------
ImTextureID OffscreenCam::imguiTexture(int frameIndex) const
{
    return reinterpret_cast<ImTextureID>(m_imguiSets[frameIndex]);
}
