#include "apeiron/render/Geometry.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

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
            // Equirectangular UV: u goes [0,1] along longitude, v along latitude.
            glm::vec2 uv{ static_cast<float>(s) / static_cast<float>(sectors),
                          static_cast<float>(r) / static_cast<float>(rings) };
            vertices.push_back({ n * radius, n, color, uv });
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

std::pair<std::vector<Vertex>, std::vector<uint32_t>>
loadObj(const std::filesystem::path& path, glm::vec3 color)
{
    tinyobj::ObjReaderConfig cfg;
    cfg.triangulate = true;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(path.string(), cfg))
        throw std::runtime_error("OBJ load failed: " + path.string()
                                 + "\n" + reader.Error());
    if (!reader.Warning().empty())
        std::cerr << "OBJ warning (" << path.filename().string() << "): "
                  << reader.Warning() << "\n";

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();

    const bool hasSrcNormals = !attrib.normals.empty();
    const bool hasSrcUVs     = !attrib.texcoords.empty();

    // Flatten all shapes into independent triangle vertices (no dedup).
    // Shape models are small enough that the redundancy doesn't matter.
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> uvs;
    positions.reserve(attrib.vertices.size());

    for (auto& shape : shapes) {
        for (auto& idx : shape.mesh.indices) {
            positions.push_back({
                attrib.vertices[3 * idx.vertex_index + 0],
                attrib.vertices[3 * idx.vertex_index + 1],
                attrib.vertices[3 * idx.vertex_index + 2]
            });
            if (hasSrcNormals && idx.normal_index >= 0) {
                normals.push_back({
                    attrib.normals[3 * idx.normal_index + 0],
                    attrib.normals[3 * idx.normal_index + 1],
                    attrib.normals[3 * idx.normal_index + 2]
                });
            } else {
                normals.emplace_back(0.0f, 0.0f, 0.0f);  // filled below
            }
            if (hasSrcUVs && idx.texcoord_index >= 0) {
                uvs.push_back({
                    attrib.texcoords[2 * idx.texcoord_index + 0],
                    1.0f - attrib.texcoords[2 * idx.texcoord_index + 1]  // flip V
                });
            } else {
                uvs.emplace_back(0.0f, 0.0f);  // filled below
            }
        }
    }

    const auto n = positions.size();
    if (n == 0)
        throw std::runtime_error("OBJ file contains no geometry: " + path.string());

    // Centre at centroid, then scale to unit bounding sphere.
    glm::vec3 centroid{0.0f};
    for (auto& p : positions) centroid += p;
    centroid /= static_cast<float>(n);
    for (auto& p : positions) p -= centroid;

    float maxDist = 0.0f;
    for (auto& p : positions) maxDist = std::max(maxDist, glm::length(p));
    if (maxDist > 0.0f)
        for (auto& p : positions) p /= maxDist;

    // Compute flat face normals for any vertex that lacks one.
    if (!hasSrcNormals) {
        for (size_t i = 0; i + 2 < n; i += 3) {
            glm::vec3 fn = glm::normalize(
                glm::cross(positions[i + 1] - positions[i],
                           positions[i + 2] - positions[i]));
            normals[i] = normals[i + 1] = normals[i + 2] = fn;
        }
    }

    // Generate equirectangular spherical UVs (poles along ±Z) where absent.
    if (!hasSrcUVs) {
        constexpr float pi = std::numbers::pi_v<float>;
        for (size_t i = 0; i < n; ++i) {
            glm::vec3 nd = glm::normalize(positions[i]);
            float u = std::atan2(nd.y, nd.x) / (2.0f * pi);
            if (u < 0.0f) u += 1.0f;
            float v = std::acos(std::clamp(nd.z, -1.0f, 1.0f)) / pi;
            uvs[i] = {u, v};
        }
    }

    // Assemble Vertex array with a trivial sequential index buffer.
    std::vector<Vertex>   vertices(n);
    std::vector<uint32_t> indices(n);
    for (size_t i = 0; i < n; ++i) {
        vertices[i] = { positions[i], glm::normalize(normals[i]), color, uvs[i] };
        indices[i]  = static_cast<uint32_t>(i);
    }

    std::cout << "Loaded OBJ " << path.filename().string()
              << ": " << (n / 3) << " triangles\n" << std::flush;
    return {vertices, indices};
}

std::pair<std::vector<Vertex>, std::vector<uint32_t>>
makeRing(float innerRadius, float outerRadius, uint32_t sectors)
{
    std::vector<Vertex>   vertices;
    std::vector<uint32_t> indices;
    vertices.reserve((sectors + 1) * 2);
    indices.reserve(sectors * 6);

    constexpr float tau = 2.0f * std::numbers::pi_v<float>;
    const glm::vec3 normal{0.0f, 0.0f, 1.0f};
    const glm::vec3 white {1.0f, 1.0f, 1.0f};

    for (uint32_t s = 0; s <= sectors; ++s) {
        float phi = tau * static_cast<float>(s) / static_cast<float>(sectors);
        float c = std::cos(phi), ss = std::sin(phi);

        // Inner vertex: u=0
        vertices.push_back({glm::vec3(innerRadius * c, innerRadius * ss, 0.0f),
                            normal, white, glm::vec2(0.0f, 0.5f)});
        // Outer vertex: u=1
        vertices.push_back({glm::vec3(outerRadius * c, outerRadius * ss, 0.0f),
                            normal, white, glm::vec2(1.0f, 0.5f)});
    }

    for (uint32_t s = 0; s < sectors; ++s) {
        uint32_t a = s * 2;      // inner  s
        uint32_t b = a + 1;      // outer  s
        uint32_t c = a + 2;      // inner  s+1
        uint32_t d = a + 3;      // outer  s+1
        indices.push_back(a); indices.push_back(b); indices.push_back(c);
        indices.push_back(c); indices.push_back(b); indices.push_back(d);
    }

    return {vertices, indices};
}

} // namespace apeiron::render::Geometry
