#pragma once

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>

#include <array>

namespace apeiron::render {

// Interleaved vertex layout: position (vec3) then color (vec3).
// Will grow to include normals and UVs once we move to lit geometry.
struct Vertex {
    glm::vec3 position;
    glm::vec3 color;

    // Single binding: all attributes in one tightly-packed buffer.
    static vk::VertexInputBindingDescription bindingDescription()
    {
        vk::VertexInputBindingDescription desc{};
        desc.setBinding  (0)
            .setStride   (sizeof(Vertex))
            .setInputRate(vk::VertexInputRate::eVertex);
        return desc;
    }

    // location 0 → position,  location 1 → color.
    static std::array<vk::VertexInputAttributeDescription, 2> attributeDescriptions()
    {
        std::array<vk::VertexInputAttributeDescription, 2> attrs{};
        attrs[0].setBinding (0)
                .setLocation(0)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, position));
        attrs[1].setBinding (0)
                .setLocation(1)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, color));
        return attrs;
    }
};

} // namespace apeiron::render
