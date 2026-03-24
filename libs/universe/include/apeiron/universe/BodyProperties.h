#pragma once

#include <string>

namespace apeiron::universe {

// Physical constants for a solar-system body.
// Radius is the IAU equatorial (a-axis) value in km, read from a loaded PCK.
struct BodyProperties {
    double radiusKm      = 0.0;  // equatorial (a-axis), km
    double polarRadiusKm = 0.0;  // polar (c-axis), km  — equals radiusKm for spheres

    // Query from the SPICE PCK kernel currently loaded in the pool.
    // Throws std::runtime_error if the body or PCK is not available.
    static BodyProperties queryFromSpice(const std::string& naifName);
};

} // namespace apeiron::universe
