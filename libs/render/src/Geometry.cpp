#include "apeiron/render/Geometry.h"

#include <glm/glm.hpp>
#include <numbers>
#include <cmath>

namespace apeiron::render::Geometry {

std::pair<std::vector<Vertex>, std::vector<uint32_t>>
makeSphere(float radius, uint32_t rings, uint32_t sectors, glm::vec3 color)
{
    std::vector<Vertex>   vertices;
    std::vector<uint32_t> indices;
    vertices.reserve((rings + 1) * (sectors + 1));

    constexpr float pi  = std::numbers::pi_v<float>;
    constexpr float tau = 2.0f * pi;

    for (uint32_t r = 0; r <= rings; ++r) {
        float theta    = pi  * static_cast<float>(r) / static_cast<float>(rings);
        float sinTheta = std::sin(theta);
        float cosTheta = std::cos(theta);

        for (uint32_t s = 0; s <= sectors; ++s) {
            float phi    = tau * static_cast<float>(s) / static_cast<float>(sectors);
            float sinPhi = std::sin(phi);
            float cosPhi = std::cos(phi);

            // Unit direction from centre.
            // Poles along +Z/-Z to match the IAU body-fixed frame convention
            // (Z = north pole), so SPICE orientation matrices apply correctly.
            glm::vec3 n{ sinTheta * cosPhi,
                         sinTheta * sinPhi,
                         cosTheta };
            vertices.push_back({ n * radius, n, color });
        }
    }

    // Two triangles per quad, winding: clockwise when viewed from outside.
    indices.reserve(rings * sectors * 6);
    uint32_t stride = sectors + 1;
    for (uint32_t r = 0; r < rings; ++r) {
        for (uint32_t s = 0; s < sectors; ++s) {
            uint32_t a = r * stride + s;
            uint32_t b = a + stride;
            // Triangle 1
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(a + 1);
            // Triangle 2
            indices.push_back(a + 1);
            indices.push_back(b);
            indices.push_back(b + 1);
        }
    }

    return {vertices, indices};
}

} // namespace apeiron::render::Geometry
