#pragma once

#include "apeiron/spacecraft/SpacecraftModel.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace spacecraft {

enum class AutopilotMode {
    Off,
    Killrot,     // damp angular velocity to zero
    Prograde,    // hold body +X along velocity vector
    Retrograde,  // hold body +X against velocity vector
    NormalPlus,  // hold body +X along orbit normal (r × v)
    NormalMinus, // hold body +X against orbit normal
    NullV,       // null relative velocity to a target body (docking mode)
    RelVelPlus,  // hold body +X along relative velocity vector (+V, docking)
    RelVelMinus, // hold body +X against relative velocity vector (-V, docking)
};

// ---------------------------------------------------------------------------
// Autopilot
//
// Killrot — three-phase bang-bang:
//   Phase 1: saturate allocator in -ω direction until |ω| < timedBurnThreshold.
//   Phase 2: timed burn — compute exact duration from I_eff × |ω| / rcsAuthorityNm.
//   Phase 3: settle clamp — zero ω directly below hardware minimum-impulse floor.
//
// Attitude hold (Prograde / Retrograde) — coast-band strategy:
//   The caller sets targetAttitude (body→inertial) and omegaFF (feedforward
//   orbital rate, body frame) each frame before calling compute().
//   A PD law gives the torque direction:
//       τ_dir = −Kp × error_angle × error_axis  −  Kd × (ω_body − ω_ff)
//   Outside the coast band (combined error > fireThreshold): saturate allocator
//   in τ_dir — bang-bang, always full authority, PD direction handles switching.
//   Inside the coast band: fire nothing — feedforward keeps attitude prograde.
//   Settle clamp: when rate error < settleClampThreshold, set ω = ω_ff directly.
//
// Parameters:
//   maxTorqueNm          N·m      Requested torque — must exceed RCS authority.
//   rcsAuthorityNm       N·m/axis Conservative actual torque (for timed burns).
//   timedBurnThreshold   rad/s    Killrot Phase 1→2 handoff.
//   settleClampThreshold rad/s    Rate residual below which clamp fires.
//   deadband             rad/s    Hysteresis below settle clamp (no re-trigger).
//   KdAtt                s        Attitude hold rate-error weight (dimensionless
//                                 when multiplied by rad/s → adds equivalent rad).
//   fireThreshold        rad      Combined PD error below which attitude hold coasts.
// ---------------------------------------------------------------------------
struct Autopilot {
    AutopilotMode mode                 = AutopilotMode::Off;

    // Shared / Killrot
    double maxTorqueNm          = 10000.0;  // N·m
    double rcsAuthorityNm       = 2500.0;   // N·m/axis
    double timedBurnThreshold   = 0.035;    // rad/s  (~2 °/s)
    double settleClampThreshold = 0.0009;   // rad/s  (~0.05 °/s)
    double deadband             = 0.0001;   // rad/s

    // Attitude hold — set by caller each frame
    glm::dquat targetAttitude   {1.0, 0.0, 0.0, 0.0}; // desired body→inertial
    glm::dvec3 omegaFF          {};                     // feedforward rate, body frame rad/s

    // Attitude hold gains / thresholds
    double KdAtt              = 15.0;    // s — cross-axis rate damping weight
    double attFireThreshold   = 0.010;   // rad  (~0.57°) — coast if att error below this
    double rateFireThreshold  = 0.003;   // rad/s (~0.17°/s) — coast if rate error below this

    // Secondary mode — runs alongside NullV.
    // When mode == NullV and secondaryMode == RelVelPlus or RelVelMinus,
    // compute() fills [3..5] from the full attitude hold rather than simple damping.
    // For all other primary modes secondaryMode should be Off.
    AutopilotMode secondaryMode = AutopilotMode::Off;

    bool active() const { return mode != AutopilotMode::Off; }

    // NullV — set by caller before enabling NullV mode.
    // relVelBody: current relative velocity in the *player body frame* (m/s).
    // Caller must update this each frame from fresh state.
    // compute() writes the desired body-frame translational force into Wrench[0..2]
    // and an angular-velocity damping torque into Wrench[3..5].
    glm::dvec3 relVelBody   {};         // m/s, player body frame
    double     nullVAuthN   = 0.0;      // N — actual bang-bang force; written by caller
                                        //     (from simulateRcsForce) for NavConsole use only.
                                        //     Not used by compute() — compute uses a saturating demand.
    double     nullVDoneThr = 0.02;     // m/s — declared done below this residual speed

    // Set by compute() each frame — true when performing a large-angle slew
    // (Phase 1 bang-bang or killrot at high rate).  Caller can use this to
    // select full vs RCS-only thruster allocation.
    bool inLargeSlew = false;

    // Set by compute() when NullV residual drops below nullVDoneThr.
    // Caller should reset mode to Off when this is true.
    bool nullVDone = false;

    // ---------------------------------------------------------------------------
    // Target update helpers — call once per frame before compute().
    // ---------------------------------------------------------------------------

    // For Prograde / Retrograde / NormalPlus / NormalMinus:
    // Computes targetAttitude (body→inertial) and omegaFF from the ship's
    // current orbital state.  No-op if mode is not one of those four.
    //   r   — geocentric position, km
    //   v   — inertial velocity,   km/s
    //   att — current body→inertial quaternion
    void updateOrbitalTarget(const glm::dvec3& r, const glm::dvec3& v,
                             const glm::dquat& att);

    // For RelVelPlus / RelVelMinus (primary or secondary alongside NullV):
    // Builds targetAttitude from relVelBody (must be set by caller first).
    // Zeros omegaFF.  No-op if neither mode applies.
    //   att — current body→inertial quaternion
    void updateRelVelTarget(const glm::dquat& att);

    // ---------------------------------------------------------------------------
    // Compute the desired wrench.
    // currentAttitude: current body→inertial quaternion (used by attitude hold).
    // omega_body:      body-frame angular velocity, rad/s.
    // inertiaDiag:     principal moments of inertia, kg·m².
    // dt:              frame time, seconds.
    // settleClamp[out]:true when the caller should set angular velocity to
    //                  omegaFF (body frame → convert to inertial before applying).
    Wrench compute(const glm::dquat& currentAttitude,
                   const glm::dvec3& omega_body,
                   const glm::dvec3& inertiaDiag,
                   double            dt,
                   bool&             settleClamp);

private:
    // Killrot timed-burn state
    bool       m_timedBurnActive = false;
    double     m_burnTimer       = 0.0;
    glm::dvec3 m_burnTorqueDir   {};
};

} // namespace spacecraft
