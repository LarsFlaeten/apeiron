#pragma once

#include "apeiron/spacecraft/SpacecraftModel.h"

#include <glm/glm.hpp>

namespace spacecraft {

enum class AutopilotMode {
    Off,
    Killrot,    // damp angular velocity to zero
};

// ---------------------------------------------------------------------------
// Autopilot — two-phase bang-bang killrot
//
// Phase 1 — Bang-bang: while |ω| > timedBurnThreshold, request maxTorqueNm
//   in the -ω direction.  The allocator saturates → full thruster firings.
//
// Phase 2 — Timed burn: once |ω| drops below timedBurnThreshold, compute the
//   exact burn duration needed to null the residual:
//       t_burn = I_eff × |ω| / rcsAuthorityNm
//   Fire for exactly t_burn seconds, then stop.  Any leftover residual
//   (from inertia estimate error) re-triggers Phase 2 automatically.
//
//   maxTorqueNm        N·m    Requested torque — must exceed RCS authority
//                             so the allocator always saturates (~10 000 for
//                             Orion whose RCS delivers ~3 000 N·m/axis).
//   rcsAuthorityNm     N·m    Conservative estimate of actual achieved torque
//                             per axis.  Under-estimating is safer (slightly
//                             long burn) than over-estimating (under-burn).
//   timedBurnThreshold rad/s  |ω| at which Phase 1 hands off to Phase 2.
//   deadband           rad/s  Below this |ω| no torque is commanded at all.
// ---------------------------------------------------------------------------
struct Autopilot {
    AutopilotMode mode                 = AutopilotMode::Off;
    double        maxTorqueNm          = 10000.0;  // N·m (saturates allocator)
    double        rcsAuthorityNm       = 2500.0;   // N·m/axis (conservative)
    double        timedBurnThreshold   = 0.035;    // rad/s (~2 °/s)
    double        settleClampThreshold = 0.0009;   // rad/s (~0.05 °/s) — below
                                                   //   this the caller zeros ω
    double        deadband             = 0.0001;   // rad/s — hysteresis gap so
                                                   //   clamp doesn't re-trigger

    bool active() const { return mode != AutopilotMode::Off; }

    // Compute the desired wrench for the current body-frame angular velocity.
    // dt: frame time in seconds (used by the timed-burn phase).
    // inertiaDiag: principal moments of inertia in kg·m² (from SpacecraftModel).
    // settleClamp [out]: set true when |ω| < deadband and the caller should
    //   zero angular velocity directly (hardware minimum-impulse floor).
    // Resets internal state when mode is Off.
    Wrench compute(const glm::dvec3& omega_body,
                   const glm::dvec3& inertiaDiag,
                   double            dt,
                   bool&             settleClamp);

private:
    bool       m_timedBurnActive = false;
    double     m_burnTimer       = 0.0;   // remaining burn time, seconds
    glm::dvec3 m_burnTorqueDir   {};      // unit vector opposing ω at burn start
};

} // namespace spacecraft
