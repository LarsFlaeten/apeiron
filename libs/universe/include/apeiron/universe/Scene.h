#pragma once

#include "apeiron/universe/FloatingOrigin.h"

#include "astro/Time.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace apeiron::universe {

class SceneNode;
class CelestialBody;

// Top-level container for all simulated objects.
//
// Owns a collection of SceneNodes and a FloatingOrigin. Each call to update()
// recenters the origin around the camera, then ticks every node so their
// worldPositions are current. renderPosition() on any node then gives a
// float-safe GPU-ready offset.
class Scene {
public:
    // Take ownership of an arbitrary SceneNode; returns a reference to it.
    SceneNode& addNode(std::unique_ptr<SceneNode> node);

    // Convenience factory: create and add a CelestialBody.
    CelestialBody& addBody(std::string name,
                           std::string naifName,
                           double      radiusKm,
                           std::string observer = "SOLAR SYSTEM BARYCENTER");

    // Recenter origin at cameraWorldPos, then update all nodes for time et.
    void update(const astro::EphemerisTime& et, const glm::dvec3& cameraWorldPos);

    const FloatingOrigin&                          origin() const;
    FloatingOrigin&                                origin();
    const std::vector<std::unique_ptr<SceneNode>>& nodes()  const;

private:
    FloatingOrigin                          m_origin;
    std::vector<std::unique_ptr<SceneNode>> m_nodes;
};

} // namespace apeiron::universe
