#pragma once

#include "apeiron/render/Buffer.h"
#include "apeiron/render/Vertex.h"

#include <vulkan/vulkan.hpp>
#include <span>
#include <cstdint>

namespace apeiron::render {

class GpuAllocator;

// Holds a device-local vertex buffer and index buffer for a piece of geometry.
//
// Data is uploaded via a staging buffer in the constructor (immediateSubmit).
// After construction the CPU-side data is no longer needed.
class Mesh {
public:
    Mesh(GpuAllocator&            allocator,
         std::span<const Vertex>  vertices,
         std::span<const uint32_t> indices);

    Mesh(Mesh&&)            = default;
    Mesh(const Mesh&)       = delete;
    Mesh& operator=(Mesh&&) = default;
    Mesh& operator=(const Mesh&) = delete;

    // Bind buffers and issue a drawIndexed call.
    void draw(vk::CommandBuffer cmd) const;

    uint32_t indexCount()  const { return m_indexCount; }
    uint32_t vertexCount() const { return m_vertexCount; }

private:
    Buffer   m_vertexBuffer;
    Buffer   m_indexBuffer;
    uint32_t m_indexCount{};
    uint32_t m_vertexCount{};
};

} // namespace apeiron::render
