#pragma once

#include <string>
#include <glm/glm.hpp>

#include "astro/Time.h"

namespace apeiron::universe {

class FloatingOrigin;

// Base class for anything that lives in the simulated universe.
//
// Stores a 64-bit world position in km relative to the Solar System
// Barycenter (SSB). Subclasses override update() to drive that position
// from physics or ephemeris each frame.
class SceneNode {
public:
    explicit SceneNode(std::string name);
    virtual ~SceneNode() = default;

    const std::string& name()          const;
    const glm::dvec3&  worldPosition() const;

    // Set position explicitly (e.g. for spacecraft or static markers).
    void setPosition(const glm::dvec3& pos);

    // Convert world position to a 32-bit render offset via the floating origin.
    glm::vec3 renderPosition(const FloatingOrigin& origin) const;

    // Override to pull a fresh position from ephemeris or a propagator.
    // Default implementation is a no-op (static node).
    virtual void update(const astro::EphemerisTime& et);

private:
    std::string m_name;
    glm::dvec3  m_worldPosition{0.0, 0.0, 0.0};
};

} // namespace apeiron::universe
