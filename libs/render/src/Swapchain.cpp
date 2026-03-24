#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/GpuAllocator.h"

#include <stdexcept>

namespace apeiron::render {

vk::Format Swapchain::findDepthFormat() const
{
    for (auto candidate : {vk::Format::eD32Sfloat,
                           vk::Format::eD32SfloatS8Uint,
                           vk::Format::eD24UnormS8Uint})
    {
        auto props = m_ctx.physicalDevice().getFormatProperties(candidate);
        if (props.optimalTilingFeatures &
            vk::FormatFeatureFlagBits::eDepthStencilAttachment)
            return candidate;
    }
    throw std::runtime_error("No supported depth format found");
}

void Swapchain::buildSwapchain(uint32_t width, uint32_t height)
{
    vkb::SwapchainBuilder builder(m_ctx.vkbDevice());
    // Pass the old swapchain so the driver can reuse resources where possible.
    if (m_vkbSwapchain.swapchain != VK_NULL_HANDLE)
        builder.set_old_swapchain(m_vkbSwapchain);

    auto result = builder
        .set_desired_format({VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
        .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
        .set_desired_extent(width, height)
        .build();
    if (!result)
        throw std::runtime_error("Swapchain creation failed: " + result.error().message());

    // Destroy old swapchain (safe after set_old_swapchain hands it off).
    if (m_vkbSwapchain.swapchain != VK_NULL_HANDLE) {
        for (auto v : m_imageViews)
            m_ctx.device().destroyImageView(v);
        m_imageViews.clear();
        vkb::destroy_swapchain(m_vkbSwapchain);
    }

    m_vkbSwapchain = result.value();

    auto views = m_vkbSwapchain.get_image_views();
    if (!views)
        throw std::runtime_error("Failed to get swapchain image views: "
                                 + views.error().message());
    for (auto v : views.value())
        m_imageViews.push_back(vk::ImageView(v));
}

void Swapchain::createImages(uint32_t width, uint32_t height)
{
    vk::Format depthFmt = findDepthFormat();
    m_depthImage = Image(m_allocator->handle(),
                         vk::Extent2D{width, height},
                         depthFmt,
                         vk::ImageUsageFlagBits::eDepthStencilAttachment);

    vk::ImageViewCreateInfo viewInfo{};
    viewInfo.setImage           (m_depthImage.handle())
            .setViewType        (vk::ImageViewType::e2D)
            .setFormat          (depthFmt)
            .setSubresourceRange({vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1});
    m_depthView = m_ctx.device().createImageView(viewInfo);

    for (int i = 0; i < kHdrCount; ++i) {
        m_hdrImages[i] = Image(m_allocator->handle(),
                               vk::Extent2D{width, height},
                               kHdrFormat,
                               vk::ImageUsageFlagBits::eColorAttachment |
                               vk::ImageUsageFlagBits::eSampled);

        vk::ImageViewCreateInfo hdrViewInfo{};
        hdrViewInfo.setImage           (m_hdrImages[i].handle())
                   .setViewType        (vk::ImageViewType::e2D)
                   .setFormat          (kHdrFormat)
                   .setSubresourceRange({vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1});
        m_hdrImageViews[i] = m_ctx.device().createImageView(hdrViewInfo);
    }
}

void Swapchain::destroyImages()
{
    auto device = m_ctx.device();
    for (auto view : m_hdrImageViews)
        device.destroyImageView(view);
    // m_hdrImages destroyed by Image RAII when overwritten
    device.destroyImageView(m_depthView);
    // m_depthImage destroyed by Image RAII when overwritten
}

Swapchain::Swapchain(const Context& ctx, GpuAllocator& allocator,
                     uint32_t width, uint32_t height)
    : m_ctx(ctx), m_allocator(&allocator)
{
    buildSwapchain(width, height);
    auto ext = extent();
    createImages(ext.width, ext.height);
}

Swapchain::~Swapchain()
{
    destroyImages();
    for (auto view : m_imageViews)
        m_ctx.device().destroyImageView(view);
    vkb::destroy_swapchain(m_vkbSwapchain);
}

void Swapchain::recreate(uint32_t width, uint32_t height)
{
    destroyImages();
    buildSwapchain(width, height);
    auto ext = extent();
    createImages(ext.width, ext.height);
}

} // namespace apeiron::render
