#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>

namespace apeiron::render {

// A 2-D GPU image backed by a VMA allocation.  Movable but not copyable.
// Mirrors the Buffer pattern: owns the VkImage + VmaAllocation;
// image views are created separately by the owner (e.g. Swapchain).
class Image {
public:
    Image() = default;
    Image(VmaAllocator         allocator,
          vk::Extent2D         extent,
          vk::Format           format,
          vk::ImageUsageFlags  usage,
          VmaMemoryUsage       memUsage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
    ~Image();

    Image(Image&&) noexcept;
    Image& operator=(Image&&) noexcept;

    Image(const Image&)            = delete;
    Image& operator=(const Image&) = delete;

    vk::Image  handle() const { return vk::Image(m_image); }
    vk::Format format() const { return m_format;           }
    bool       valid()  const { return m_image != VK_NULL_HANDLE; }

private:
    VmaAllocator  m_allocator{};
    VkImage       m_image{VK_NULL_HANDLE};
    VmaAllocation m_allocation{};
    vk::Format    m_format{};
};

} // namespace apeiron::render
