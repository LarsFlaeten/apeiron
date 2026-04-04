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
// Killrot uses a bang-bang strategy: while |ω| > deadband, command the
// maximum counter-torque in the -ω direction.  The allocator saturates the
// thrusters, producing full-strength discrete firings rather than the
// fractional throttles of proportional control (which are too small for the
// PWM cycle to produce visible or effective burns).
//
//   maxTorqueNm  N·m   Requested torque magnitude — set well above the
//                      vehicle's actual RCS authority so the allocator always
//                      saturates.  For Orion RCS (~3 000 N·m/axis) 10 000 is
//                      a safe ceiling.
//   deadband     rad/s Below this |ω| no torque is commanded.
// ---------------------------------------------------------------------------
struct Autopilot {
    AutopilotMode mode         = AutopilotMode::Off;
    double        maxTorqueNm  = 10000.0;  // N·m  (deliberately > RCS authority)
    double        deadband     = 0.005;    // rad/s (~0.29 °/s)

    bool active() const { return mode != AutopilotMode::Off; }

    // Compute the desired wrench for the current body-frame angular velocity.
    // Returns a zero wrench when Off or |ω| < deadband.  Does not modify mode.
    Wrench compute(const glm::dvec3& omega_body) const;
};

} // namespace spacecraft
