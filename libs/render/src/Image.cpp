#include "apeiron/render/Image.h"

#include <stdexcept>

namespace apeiron::render {

Image::Image(VmaAllocator        allocator,
             vk::Extent2D        extent,
             vk::Format          format,
             vk::ImageUsageFlags usage,
             VmaMemoryUsage      memUsage)
    : m_allocator(allocator), m_format(format)
{
    VkImageCreateInfo imageInfo{};
    imageInfo.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType     = VK_IMAGE_TYPE_2D;
    imageInfo.extent        = {extent.width, extent.height, 1};
    imageInfo.mipLevels     = 1;
    imageInfo.arrayLayers   = 1;
    imageInfo.format        = static_cast<VkFormat>(format);
    imageInfo.tiling        = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage         = static_cast<VkImageUsageFlags>(usage);
    imageInfo.samples       = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = memUsage;

    if (vmaCreateImage(m_allocator, &imageInfo, &allocInfo,
                       &m_image, &m_allocation, nullptr) != VK_SUCCESS)
        throw std::runtime_error("Failed to create VMA image");
}

Image::~Image()
{
    if (m_image != VK_NULL_HANDLE)
        vmaDestroyImage(m_allocator, m_image, m_allocation);
}

Image::Image(Image&& o) noexcept
    : m_allocator(o.m_allocator)
    , m_image    (o.m_image)
    , m_allocation(o.m_allocation)
    , m_format   (o.m_format)
{
    o.m_image      = VK_NULL_HANDLE;
    o.m_allocation = VK_NULL_HANDLE;
}

Image& Image::operator=(Image&& o) noexcept
{
    if (this != &o) {
        if (m_image != VK_NULL_HANDLE)
            vmaDestroyImage(m_allocator, m_image, m_allocation);
        m_allocator  = o.m_allocator;
        m_image      = o.m_image;
        m_allocation = o.m_allocation;
        m_format     = o.m_format;
        o.m_image      = VK_NULL_HANDLE;
        o.m_allocation = VK_NULL_HANDLE;
    }
    return *this;
}

} // namespace apeiron::render
