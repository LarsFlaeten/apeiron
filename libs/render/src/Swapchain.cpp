#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Context.h"

#include <stdexcept>

namespace apeiron::render {

Swapchain::Swapchain(const Context& ctx, uint32_t width, uint32_t height)
    : m_ctx(ctx)
{
    vkb::SwapchainBuilder builder(ctx.vkbDevice());
    auto result = builder
        .set_desired_format({VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
        .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)   // vsync, always supported
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
}

Swapchain::~Swapchain()
{
    auto device = m_ctx.device();
    for (auto view : m_imageViews)
        device.destroyImageView(view);
    vkb::destroy_swapchain(m_vkbSwapchain);
}

} // namespace apeiron::render
