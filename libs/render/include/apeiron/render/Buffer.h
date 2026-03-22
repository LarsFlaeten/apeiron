#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>

namespace apeiron::render {

// A GPU buffer backed by a VMA allocation.  Movable but not copyable.
//
// Usage flags control what the buffer can be used for (vertex, index,
// transfer source/destination, …).  memUsage and allocFlags control
// where VMA places the allocation (device-local, host-visible, etc.).
class Buffer {
public:
    Buffer() = default;
    Buffer(VmaAllocator        allocator,
           vk::DeviceSize      size,
           vk::BufferUsageFlags usage,
           VmaMemoryUsage      memUsage,
           VmaAllocationCreateFlags allocFlags = 0);
    ~Buffer();

    Buffer(Buffer&&) noexcept;
    Buffer& operator=(Buffer&&) noexcept;

    Buffer(const Buffer&)            = delete;
    Buffer& operator=(const Buffer&) = delete;

    vk::Buffer     handle() const { return vk::Buffer(m_buffer); }
    vk::DeviceSize size()   const { return m_size; }
    bool           valid()  const { return m_buffer != VK_NULL_HANDLE; }

    // Map, memcpy bytes into the allocation, then unmap.
    // Only valid for host-visible allocations (staging buffers).
    void upload(const void* data, vk::DeviceSize bytes);

private:
    VmaAllocator  m_allocator{};
    VkBuffer      m_buffer{VK_NULL_HANDLE};
    VmaAllocation m_allocation{};
    vk::DeviceSize m_size{};
};

} // namespace apeiron::render
