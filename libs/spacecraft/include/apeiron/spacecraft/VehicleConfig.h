#pragma once

#include "SpacecraftModel.h"
#include "astro/OrbitElements.h"
#include "astro/Time.h"
#include "astro/State.h"

#include <filesystem>
#include <string>

namespace spacecraft {

// ---------------------------------------------------------------------------
// VehicleConfig — vehicle definition loaded from a spacecraft TOML.
//
// File format:
//
//   [vehicle]                   # required
//   name           = "Orion"
//   mass_kg        = 26500.0
//   center_of_mass = [-0.5, 0.0, 0.0]   # optional
//   inertia_tensor = [34000.0, 120000.0, 120000.0]
//   glb            = "spacecraft/orion/CM_SM06.glb"  # optional
//
//   [[thrusters]]               # zero or more; absent for passive craft
//   id           = "main"
//   type         = "main_engine"
//   thrust_n     = 25700.0
//   ...
//
// Orbit information is NOT stored in spacecraft files — it is supplied by the
// scenario and applied via applyOrbit() after SPICE kernels are loaded.
// ---------------------------------------------------------------------------

struct VehicleConfig {
    std::string   name;
    float         massKg       = 1.0f;
    glm::vec3     centerOfMass {};
    glm::dvec3    inertiaDiag  {};      // principal moments, kg·m²
    std::string   glbPath;              // relative to APEIRON_DATA_DIR; empty = no model

    bool                  hasOrbit     = false;
    std::string           centralBody  = "EARTH";  // SPICE name of the body being orbited
    astro::OrbitElements  orbitElements;   // valid when hasOrbit == true

    SpacecraftModel       model;           // thrusters; empty if no [[thrusters]] section
};

// Parse a spacecraft TOML (vehicle + thrusters only).
// Throws std::runtime_error on bad input.
VehicleConfig loadVehicleConfig(const std::filesystem::path& tomlPath);

// Apply scenario-supplied orbit parameters to a VehicleConfig.
// Requires SPICE kernels to be loaded (queries GM via SPICE).
// Throws std::runtime_error if centralBody is unknown or GM unavailable.
void applyOrbit(VehicleConfig&     vc,
                const std::string& centralBody,
                const std::string& epochStr,
                double             incDeg,
                double             raanDeg,
                double             eccentricity,
                double             argPerigeeDeg,
                double             meanAnomalyDeg,
                double             meanMotionRevDay);

// Propagate VehicleConfig orbit elements to an ECI (ECLIPJ2000) state at et.
// Requires hasOrbit == true.
astro::PosState vehicleStateAtEt(const VehicleConfig& vc,
                                  const astro::EphemerisTime& et);

} // namespace spacecraft
