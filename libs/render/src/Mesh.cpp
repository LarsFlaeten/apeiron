#include "apeiron/render/Mesh.h"
#include "apeiron/render/GpuAllocator.h"

namespace apeiron::render {

Mesh::Mesh(GpuAllocator&             allocator,
           std::span<const Vertex>   vertices,
           std::span<const uint32_t> indices)
    : m_indexCount (static_cast<uint32_t>(indices.size()))
    , m_vertexCount(static_cast<uint32_t>(vertices.size()))
{
    const vk::DeviceSize vbSize = vertices.size_bytes();
    const vk::DeviceSize ibSize = indices.size_bytes();

    // --- Staging buffers (CPU-visible, write-once) ---
    Buffer stagingVB(allocator.handle(), vbSize,
                     vk::BufferUsageFlagBits::eTransferSrc,
                     VMA_MEMORY_USAGE_AUTO,
                     VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT);
    stagingVB.upload(vertices.data(), vbSize);

    Buffer stagingIB(allocator.handle(), ibSize,
                     vk::BufferUsageFlagBits::eTransferSrc,
                     VMA_MEMORY_USAGE_AUTO,
                     VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT);
    stagingIB.upload(indices.data(), ibSize);

    // --- Device-local buffers ---
    m_vertexBuffer = Buffer(allocator.handle(), vbSize,
                            vk::BufferUsageFlagBits::eVertexBuffer |
                            vk::BufferUsageFlagBits::eTransferDst,
                            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);

    m_indexBuffer  = Buffer(allocator.handle(), ibSize,
                            vk::BufferUsageFlagBits::eIndexBuffer |
                            vk::BufferUsageFlagBits::eTransferDst,
                            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);

    // --- Upload: staging → device-local ---
    allocator.immediateSubmit([&](vk::CommandBuffer cmd) {
        cmd.copyBuffer(stagingVB.handle(), m_vertexBuffer.handle(),
                       vk::BufferCopy{0, 0, vbSize});
        cmd.copyBuffer(stagingIB.handle(), m_indexBuffer.handle(),
                       vk::BufferCopy{0, 0, ibSize});
    });
    // stagingVB, stagingIB destroyed here — device copy is complete.
}

void Mesh::draw(vk::CommandBuffer cmd) const
{
    vk::Buffer     vb     = m_vertexBuffer.handle();
    vk::DeviceSize offset = 0;
    cmd.bindVertexBuffers(0, vb, offset);
    cmd.bindIndexBuffer(m_indexBuffer.handle(), 0, vk::IndexType::eUint32);
    cmd.drawIndexed(m_indexCount, 1, 0, 0, 0);
}

} // namespace apeiron::render
