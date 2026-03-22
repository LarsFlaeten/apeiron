#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/GpuAllocator.h"

#include <stdexcept>

namespace apeiron::render {

// Pick the best depth format the physical device supports for optimal tiling.
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

Swapchain::Swapchain(const Context& ctx, GpuAllocator& allocator,
                     uint32_t width, uint32_t height)
    : m_ctx(ctx)
{
    // ----- Colour swapchain -----
    vkb::SwapchainBuilder builder(ctx.vkbDevice());
    auto result = builder
        .set_desired_format({VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
        .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
        .set_desired_extent(width, height)
        .build();
    if (!result)
        throw std::runtime_error("Swapchain creation failed: " + result.error().message());
    m_vkbSwapchain = result.value();

    auto views = m_vkbSwapchain.get_image_views();
    if (!views)
        throw std::runtime_error("Failed to get swapchain image views: "
                                 + views.error().message());
    for (auto v : views.value())
        m_imageViews.push_back(vk::ImageView(v));

    // ----- Depth image -----
    vk::Format depthFmt = findDepthFormat();
    m_depthImage = Image(allocator.handle(),
                         vk::Extent2D{width, height},
                         depthFmt,
                         vk::ImageUsageFlagBits::eDepthStencilAttachment);

    vk::ImageViewCreateInfo viewInfo{};
    viewInfo.setImage           (m_depthImage.handle())
            .setViewType        (vk::ImageViewType::e2D)
            .setFormat          (depthFmt)
            .setSubresourceRange({vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1});
    m_depthView = ctx.device().createImageView(viewInfo);
}

Swapchain::~Swapchain()
{
    auto device = m_ctx.device();
    device.destroyImageView(m_depthView);
    // m_depthImage destroyed by Image RAII
    for (auto view : m_imageViews)
        device.destroyImageView(view);
    vkb::destroy_swapchain(m_vkbSwapchain);
}

} // namespace apeiron::render
