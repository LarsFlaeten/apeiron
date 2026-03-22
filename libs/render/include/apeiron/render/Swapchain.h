#pragma once

#include "apeiron/render/Image.h"

#include <vulkan/vulkan.hpp>
#include <VkBootstrap.h>

#include <vector>

namespace apeiron::render {

class Context;
class GpuAllocator;

// Owns the VkSwapchainKHR, its per-image colour views, and the shared
// depth image + view (same dimensions, same lifetime).
// Framebuffers live in Renderer (they couple these views to a render pass).
class Swapchain {
public:
    Swapchain(const Context& ctx, GpuAllocator& allocator,
              uint32_t width, uint32_t height);
    ~Swapchain();

    Swapchain(const Swapchain&)            = delete;
    Swapchain& operator=(const Swapchain&) = delete;

    vk::SwapchainKHR                  handle()         const { return vk::SwapchainKHR(m_vkbSwapchain.swapchain); }
    vk::Format                        imageFormat()    const { return vk::Format(m_vkbSwapchain.image_format);    }
    vk::Extent2D                      extent()         const { return {m_vkbSwapchain.extent.width,
                                                                        m_vkbSwapchain.extent.height};             }
    const std::vector<vk::ImageView>& imageViews()     const { return m_imageViews;     }
    uint32_t                          imageCount()     const { return static_cast<uint32_t>(m_imageViews.size()); }

    vk::ImageView                     depthImageView() const { return m_depthView;       }
    vk::Format                        depthFormat()    const { return m_depthImage.format(); }

private:
    vk::Format findDepthFormat() const;

    const Context&             m_ctx;
    vkb::Swapchain             m_vkbSwapchain{};
    std::vector<vk::ImageView> m_imageViews;

    Image         m_depthImage;
    vk::ImageView m_depthView;
};

} // namespace apeiron::render
