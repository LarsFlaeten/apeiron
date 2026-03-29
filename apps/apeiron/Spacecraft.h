#pragma once

#include "astro/ODE.h"
#include "astro/PCDM.h"
#include "astro/RKF78.h"
#include "astro/State.h"
#include "astro/Time.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <optional>
#include <cstddef>

// ---------------------------------------------------------------------------
// Spacecraft
//
// Self-contained rigid body in an inertial reference frame (ECLIPJ2000 or
// Earth-centred ECI, caller's choice).  Translational state is propagated
// with RKF78, rotational state with PCDM.
//
// Coordinate / unit conventions:
//   Position          km   (matches astro/ODE.h internals)
//   Velocity          km/s
//   Force input       N    (converted internally to km-consistent units)
//   Torque input      N·m
//   Mass              kg
//   Inertia tensor    kg·m²
//   Angular velocity  rad/s (inertial frame)
//
// Attractors must be supplied in the same frame and units as the state.
// For Earth-centred coordinates, add one attractor at (0,0,0) with
//   GM = 398600.4418  (km³/s²)
// ---------------------------------------------------------------------------
class Spacecraft {
public:
    Spacecraft(double             massKg,
               const glm::dmat3& inertiaKgM2,
               const astro::State& initialState);

    // Add a gravitational attractor (position in km, GM in km³/s²).
    void addAttractor(const astro::Attractor& a);

    // Set instantaneous applied force/torque (body frame, SI units).
    // Call before each update() step.  Both default to zero.
    void setBodyForce (const glm::dvec3& forceN);   // Newtons
    void setBodyTorque(const glm::dvec3& torqueNm); // Newton-metres

    // Advance dynamics by dt_s seconds of simulation time.
    void update(double dt_s, const astro::EphemerisTime& et);

    // State accessors.
    glm::dvec3 position()        const { return m_state.P.r; }
    glm::dvec3 velocity()        const { return m_state.P.v; }
    glm::dquat attitude()        const { return m_state.R.q; }  // body→inertial
    glm::dvec3 angularVelocity() const { return m_state.R.w; }  // inertial frame

    // Derived quantities.
    double altitudeKm(double bodyRadiusKm) const
        { return glm::length(m_state.P.r) - bodyRadiusKm; }

    // Render interpolation.
    // Call saveSnapshot() once before the physics sub-step loop each frame,
    // then renderPosition/renderAttitude(alpha) where alpha = physAccum / kPhysStep.
    // This blends the last completed step with the current state for smooth rendering.
    void       saveSnapshot() { m_snapPos = m_state.P.r; m_snapAtt = m_state.R.q; }
    glm::dvec3 renderPosition(double alpha) const { return glm::mix(m_snapPos, m_state.P.r, alpha); }
    glm::dquat renderAttitude(double alpha) const { return glm::slerp(m_snapAtt, m_state.R.q, alpha); }

    // Parent-child hierarchy.
    // When set, this spacecraft is docked to another; physics is driven by parent.
    void setParent(size_t idx)  { m_parentId = idx; }
    void clearParent()          { m_parentId.reset(); }
    std::optional<size_t> parentId() const { return m_parentId; }

private:
    double          m_massKg;
    astro::State    m_state;
    astro::ODE      m_ode;
    astro::RotODE   m_rotOde;

    // Adaptive step hint carried between update() calls.
    astro::TimeDelta m_dtHint;

    std::optional<size_t> m_parentId;

    // Snapshot taken before each frame's physics loop (for render interpolation).
    glm::dvec3 m_snapPos{0.0};
    glm::dquat m_snapAtt{1.0, 0.0, 0.0, 0.0};
};
