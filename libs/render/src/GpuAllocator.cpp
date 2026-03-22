// VMA_IMPLEMENTATION must be defined in exactly one translation unit.
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

#include "apeiron/render/GpuAllocator.h"
#include "apeiron/render/Context.h"

#include <stdexcept>

namespace apeiron::render {

GpuAllocator::GpuAllocator(const Context& ctx)
    : m_ctx(ctx)
{
    VmaAllocatorCreateInfo info{};
    info.vulkanApiVersion = VK_API_VERSION_1_3;
    info.physicalDevice   = static_cast<VkPhysicalDevice>(ctx.physicalDevice());
    info.device           = static_cast<VkDevice>(ctx.device());
    info.instance         = static_cast<VkInstance>(ctx.instance());

    if (vmaCreateAllocator(&info, &m_allocator) != VK_SUCCESS)
        throw std::runtime_error("Failed to create VMA allocator");

    // Dedicated command pool for upload operations.
    vk::CommandPoolCreateInfo poolInfo{};
    poolInfo.setQueueFamilyIndex(ctx.graphicsFamily())
            .setFlags(vk::CommandPoolCreateFlagBits::eTransient);
    m_uploadPool = ctx.device().createCommandPool(poolInfo);
}

GpuAllocator::~GpuAllocator()
{
    m_ctx.device().destroyCommandPool(m_uploadPool);
    vmaDestroyAllocator(m_allocator);
}

void GpuAllocator::immediateSubmit(std::function<void(vk::CommandBuffer)> fn)
{
    auto device = m_ctx.device();

    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setCommandPool       (m_uploadPool)
             .setLevel             (vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(1);

    auto cmds = device.allocateCommandBuffers(allocInfo);
    auto cmd  = cmds[0];

    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.setFlags(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
    cmd.begin(beginInfo);
    fn(cmd);
    cmd.end();

    vk::SubmitInfo submitInfo{};
    submitInfo.setCommandBuffers(cmd);
    m_ctx.graphicsQueue().submit(submitInfo);
    m_ctx.graphicsQueue().waitIdle();

    device.freeCommandBuffers(m_uploadPool, cmd);
}

} // namespace apeiron::render
