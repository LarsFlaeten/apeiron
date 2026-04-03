#pragma once

#include "apeiron/spacecraft/SpacecraftModel.h"

#include <glm/glm.hpp>

namespace spacecraft {

enum class AutopilotMode {
    Off,
    Killrot,    // damp angular velocity to zero
};

// ---------------------------------------------------------------------------
// Autopilot
//
// Computes a desired Wrench each frame from the current body-frame angular
// velocity.  The caller feeds the result straight into SpacecraftModel::
// solveAllocation() — the autopilot knows nothing about actuators.
//
// Gains:
//   kdRot    N·m·s/rad   Damping gain.  Natural value ≈ I / τ_settle where
//                        τ_settle is the desired e-folding time in seconds.
//                        For Orion (I ≈ 100 000 kg·m²) and τ = 5 s → 20 000.
//   deadband rad/s       Below this |ω| no torque is commanded (prevents
//                        chatter when nearly stopped).
// ---------------------------------------------------------------------------
struct Autopilot {
    AutopilotMode mode     = AutopilotMode::Off;
    double        kdRot    = 20000.0;   // N·m·s/rad
    double        deadband = 0.002;     // rad/s  (~0.11 °/s)

    bool active() const { return mode != AutopilotMode::Off; }

    // Compute the desired wrench for the current body-frame angular velocity.
    // Returns a zero wrench when Off or |ω| < deadband.
    // Returns desired wrench. May modify mode (auto-disengage on settle).
    Wrench compute(const glm::dvec3& omega_body);
};

} // namespace spacecraft
