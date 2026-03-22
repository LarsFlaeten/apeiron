#pragma once

#include "astro/Time.h"

#include <glm/glm.hpp>
#include <string>

namespace apeiron::universe {

// The viewpoint from which the simulation is observed.
//
// The observer is attached to a solar-system body (e.g. "EARTH") and looks
// toward a target body (e.g. "SUN").  Both positions are queried from SPICE
// so they are always physically correct.
//
// worldPosition() is fed into Scene::update() to recenter the floating origin,
// ensuring render-space offsets remain within float precision.
class Observer {
public:
    Observer(std::string body,
             std::string target,
             std::string frame = "J2000");

    // World-space position of the body this observer is attached to (km, SSB).
    glm::dvec3 worldPosition (const astro::EphemerisTime& et) const;

    // World-space position of the target body (km, SSB).
    glm::dvec3 targetPosition(const astro::EphemerisTime& et) const;

    const std::string& body()   const { return m_body;   }
    const std::string& target() const { return m_target; }
    const std::string& frame()  const { return m_frame;  }

private:
    std::string m_body;
    std::string m_target;
    std::string m_frame;
};

} // namespace apeiron::universe
