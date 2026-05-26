#pragma once

#include "SpacecraftModel.h"
#include "astro/OrbitElements.h"
#include "astro/Time.h"
#include "astro/State.h"

#include <filesystem>
#include <string>

namespace spacecraft {

// ---------------------------------------------------------------------------
// VehicleConfig — complete description of one spacecraft loaded from TOML.
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
//   [orbit]                     # optional; present for AI/passive craft
//   epoch        = "2026-03-29T03:11:03"
//   inclination  = 51.6344      # deg
//   raan         = 336.2407     # deg
//   eccentricity = 0.0006215
//   arg_perigee  = 245.2164     # deg
//   mean_anomaly = 114.8178     # deg
//   mean_motion  = 15.48624340  # rev/day
//
//   [[thrusters]]               # zero or more; absent for passive craft
//   id           = "main"
//   type         = "main_engine"
//   thrust_n     = 25700.0
//   ...
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

// Parse a unified spacecraft TOML.  Throws std::runtime_error on bad input.
VehicleConfig loadVehicleConfig(const std::filesystem::path& tomlPath);

// Propagate VehicleConfig orbit elements to an ECLIPJ2000 state at et.
// Requires hasOrbit == true.
astro::PosState vehicleStateAtEt(const VehicleConfig& vc,
                                  const astro::EphemerisTime& et);

} // namespace spacecraft
