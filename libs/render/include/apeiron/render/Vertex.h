#pragma once

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>

#include <array>

namespace apeiron::render {

// Interleaved vertex layout: position (vec3), normal (vec3), color (vec3), uv (vec2).
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
    glm::vec2 uv;

    static vk::VertexInputBindingDescription bindingDescription()
    {
        vk::VertexInputBindingDescription desc{};
        desc.setBinding  (0)
            .setStride   (sizeof(Vertex))
            .setInputRate(vk::VertexInputRate::eVertex);
        return desc;
    }

    // location 0 → position, 1 → normal, 2 → color, 3 → uv.
    static std::array<vk::VertexInputAttributeDescription, 4> attributeDescriptions()
    {
        std::array<vk::VertexInputAttributeDescription, 4> attrs{};
        attrs[0].setBinding (0).setLocation(0)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, position));
        attrs[1].setBinding (0).setLocation(1)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, normal));
        attrs[2].setBinding (0).setLocation(2)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, color));
        attrs[3].setBinding (0).setLocation(3)
                .setFormat  (vk::Format::eR32G32Sfloat)
                .setOffset  (offsetof(Vertex, uv));
        return attrs;
    }
};

} // namespace apeiron::render
