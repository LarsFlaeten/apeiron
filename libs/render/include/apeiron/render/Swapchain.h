#pragma once

#include <vulkan/vulkan.hpp>
#include <VkBootstrap.h>

#include <vector>

namespace apeiron::render {

class Context;

// Owns the VkSwapchainKHR and its per-image views.
// Framebuffers live in Renderer (they couple the swapchain views to a render pass).
class Swapchain {
public:
    Swapchain(const Context& ctx, uint32_t width, uint32_t height);
    ~Swapchain();

    Swapchain(const Swapchain&)            = delete;
    Swapchain& operator=(const Swapchain&) = delete;

    vk::SwapchainKHR                  handle()      const { return vk::SwapchainKHR(m_vkbSwapchain.swapchain); }
    vk::Format                        imageFormat() const { return vk::Format(m_vkbSwapchain.image_format);    }
    vk::Extent2D                      extent()      const { return {m_vkbSwapchain.extent.width,
                                                                     m_vkbSwapchain.extent.height};            }
    const std::vector<vk::ImageView>& imageViews()  const { return m_imageViews; }
    uint32_t                          imageCount()  const { return static_cast<uint32_t>(m_imageViews.size()); }

private:
    const Context&             m_ctx;
    vkb::Swapchain             m_vkbSwapchain{};
    std::vector<vk::ImageView> m_imageViews;
};

} // namespace apeiron::render
