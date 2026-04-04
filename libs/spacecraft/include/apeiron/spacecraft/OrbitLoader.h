#pragma once

#include "astro/OrbitElements.h"
#include "astro/State.h"
#include "astro/Time.h"

#include <filesystem>
#include <string>

namespace spacecraft {

// Loaded from an orbit TOML file (e.g. iss_orbit.toml).
struct OrbitConfig {
    std::string            name;
    double                 massKg      = 1.0;
    glm::dvec3             inertiaDiag {};    // principal moments, kg·m²
    std::string            glbPath;           // relative to APEIRON_DATA_DIR
    astro::OrbitElements   elements;
};

// Parse an orbit TOML and return the config.
// Throws std::runtime_error on missing/malformed fields.
OrbitConfig loadOrbitConfig(const std::filesystem::path& tomlPath);

// Convenience: compute ECLIPJ2000 state vector at the given sim time.
// Handles TEME→ECLIPJ2000 rotation internally (J2000 equatorial approximation).
astro::PosState orbitStateAtEt(const astro::OrbitElements& oe,
                                const astro::EphemerisTime& et);

} // namespace spacecraft
