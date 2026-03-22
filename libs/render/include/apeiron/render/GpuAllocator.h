#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>

#include <functional>

namespace apeiron::render {

class Context;

// Wraps a VmaAllocator and owns a command pool for one-shot GPU uploads.
//
// One GpuAllocator is created per application and passed to anything that
// needs to allocate GPU memory (Buffer, Mesh, textures, …).
class GpuAllocator {
public:
    explicit GpuAllocator(const Context& ctx);
    ~GpuAllocator();

    GpuAllocator(const GpuAllocator&)            = delete;
    GpuAllocator& operator=(const GpuAllocator&) = delete;

    VmaAllocator handle() const { return m_allocator; }

    // Record GPU work into a temporary command buffer, submit it to the
    // graphics queue, and block until it completes.  Used for staging uploads
    // at load time; not for hot-path per-frame work.
    void immediateSubmit(std::function<void(vk::CommandBuffer)> fn);

private:
    const Context& m_ctx;
    VmaAllocator   m_allocator{};
    vk::CommandPool m_uploadPool;
};

} // namespace apeiron::render
