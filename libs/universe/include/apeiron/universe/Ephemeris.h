#pragma once

#include <string>
#include <glm/glm.hpp>
#include "astro/Time.h"

namespace apeiron::universe {

struct BodyState
{
    glm::dvec3 position;   ///< km, in requested frame
    glm::dvec3 velocity;   ///< km/s, in requested frame
    double      lightTime; ///< one-way light time to target, seconds
};

/// Returns the state of @p target relative to @p observer in @p frame at @p et.
///
/// @param target    NAIF body name (e.g. "MARS", "EARTH", "SUN", "EARTH BARYCENTER")
/// @param et        Ephemeris time
/// @param observer  NAIF observer name (default: "SOLAR SYSTEM BARYCENTER")
/// @param frame     Reference frame (default: "J2000")
/// @param abcorr    Aberration correction: "NONE", "LT", "LT+S", "CN", "CN+S"
///
/// Throws astro::SpiceException on SPICE error (e.g. required kernel not loaded).
BodyState getState(
    const std::string&          target,
    const astro::EphemerisTime& et,
    const std::string&          observer = "SOLAR SYSTEM BARYCENTER",
    const std::string&          frame    = "J2000",
    const std::string&          abcorr   = "NONE"
);

} // namespace apeiron::universe
