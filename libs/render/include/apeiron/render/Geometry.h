#pragma once

#include "apeiron/render/Vertex.h"

#include <glm/glm.hpp>
#include <cstdint>
#include <filesystem>
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

// Loads an OBJ file and returns vertices + indices in the same convention as
// makeSphere: mesh is centred at the origin and scaled to a unit bounding
// sphere (max vertex distance from origin = 1.0).
//
// If the OBJ has no normals, flat face normals are computed.
// If the OBJ has no UV coordinates, equirectangular spherical UVs are
// generated from the normalised vertex direction (poles along ±Z).
// color is applied as a uniform tint, as with makeSphere.
std::pair<std::vector<Vertex>, std::vector<uint32_t>>
loadObj(const std::filesystem::path& path,
        glm::vec3                    color);

} // namespace apeiron::render::Geometry
