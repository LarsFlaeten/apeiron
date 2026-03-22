#pragma once

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>

#include <array>

namespace apeiron::render {

// Interleaved vertex layout: position (vec3), normal (vec3), color (vec3).
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;

    static vk::VertexInputBindingDescription bindingDescription()
    {
        vk::VertexInputBindingDescription desc{};
        desc.setBinding  (0)
            .setStride   (sizeof(Vertex))
            .setInputRate(vk::VertexInputRate::eVertex);
        return desc;
    }

    // location 0 → position,  location 1 → normal,  location 2 → color.
    static std::array<vk::VertexInputAttributeDescription, 3> attributeDescriptions()
    {
        std::array<vk::VertexInputAttributeDescription, 3> attrs{};
        attrs[0].setBinding (0).setLocation(0)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, position));
        attrs[1].setBinding (0).setLocation(1)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, normal));
        attrs[2].setBinding (0).setLocation(2)
                .setFormat  (vk::Format::eR32G32B32Sfloat)
                .setOffset  (offsetof(Vertex, color));
        return attrs;
    }
};

} // namespace apeiron::render
