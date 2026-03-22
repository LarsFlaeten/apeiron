#include "apeiron/render/Buffer.h"

#include <cstring>
#include <stdexcept>

namespace apeiron::render {

Buffer::Buffer(VmaAllocator             allocator,
               vk::DeviceSize           size,
               vk::BufferUsageFlags     usage,
               VmaMemoryUsage           memUsage,
               VmaAllocationCreateFlags allocFlags)
    : m_allocator(allocator), m_size(size)
{
    VkBufferCreateInfo bufInfo{};
    bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufInfo.size  = size;
    bufInfo.usage = static_cast<VkBufferUsageFlags>(usage);

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = memUsage;
    allocInfo.flags = allocFlags;

    if (vmaCreateBuffer(m_allocator, &bufInfo, &allocInfo,
                        &m_buffer, &m_allocation, nullptr) != VK_SUCCESS)
        throw std::runtime_error("Failed to create VMA buffer");
}

Buffer::~Buffer()
{
    if (m_buffer != VK_NULL_HANDLE)
        vmaDestroyBuffer(m_allocator, m_buffer, m_allocation);
}

Buffer::Buffer(Buffer&& o) noexcept
    : m_allocator(o.m_allocator)
    , m_buffer    (o.m_buffer)
    , m_allocation(o.m_allocation)
    , m_size      (o.m_size)
{
    o.m_buffer     = VK_NULL_HANDLE;
    o.m_allocation = VK_NULL_HANDLE;
}

Buffer& Buffer::operator=(Buffer&& o) noexcept
{
    if (this != &o) {
        if (m_buffer != VK_NULL_HANDLE)
            vmaDestroyBuffer(m_allocator, m_buffer, m_allocation);
        m_allocator  = o.m_allocator;
        m_buffer     = o.m_buffer;
        m_allocation = o.m_allocation;
        m_size       = o.m_size;
        o.m_buffer     = VK_NULL_HANDLE;
        o.m_allocation = VK_NULL_HANDLE;
    }
    return *this;
}

void Buffer::upload(const void* data, vk::DeviceSize bytes)
{
    void* mapped{};
    vmaMapMemory(m_allocator, m_allocation, &mapped);
    std::memcpy(mapped, data, static_cast<std::size_t>(bytes));
    vmaUnmapMemory(m_allocator, m_allocation);
}

} // namespace apeiron::render
