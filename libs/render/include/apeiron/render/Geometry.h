#pragma once

#include "apeiron/render/Vertex.h"

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace apeiron::render::Geometry {

// Returns a UV-sphere centred at the origin with the given radius, rings
// (latitude bands, poles inclusive), and sectors (longitude divisions).
// Normals are the outward unit direction; color is uniform across the sphere.
std::pair<std::vector<Vertex>, std::vector<uint32_t>>
makeSphere(float     radius,
           uint32_t  rings,
           uint32_t  sectors,
           glm::vec3 color);

} // namespace apeiron::render::Geometry
