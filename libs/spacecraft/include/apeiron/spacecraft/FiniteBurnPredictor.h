#pragma once

#include <glm/glm.hpp>
#include <utility>
#include <limits>

namespace spacecraft {

// ---------------------------------------------------------------------------
// Finite-burn periapsis predictor.
//
// Numerically integrates a two-phase trajectory using RK4:
//   Phase 1 — free coast for tCoast seconds (gravity only)
//   Phase 2 — retrograde burn for tBurn seconds  (gravity + constant thrust)
//
// Designed for MOI (Mars / Jupiter / etc.) burn prediction: given the
// current arrival-body-centric state, coasts to the planned ignition point
// and simulates the full retrograde burn, then reports the predicted Pe.
// ---------------------------------------------------------------------------

// RK4-integrates (r0, v0) under mu + optional retrograde thrust.
// Returns {r_final, v_final}.
// All quantities in km and km/s.
std::pair<glm::dvec3, glm::dvec3> propagateFiniteBurn(
    glm::dvec3 r0,         // initial position  (km, body-centric)
    glm::dvec3 v0,         // initial velocity  (km/s, body-centric)
    double     mu,         // body gravitational parameter  (km³/s²)
    double     accelKmS2,  // constant thrust acceleration  (km/s²)
    double     tCoast,     // coast duration before ignition  (s)
    int        nCoastSteps,// RK4 steps for coast phase
    double     tBurn,      // burn duration  (s)
    int        nBurnSteps  // RK4 steps for burn phase
);

// Periapsis RADIUS (km from body centre) derived from (r, v, mu).
// Returns quiet_NaN() when the orbit is not captured (specific energy >= 0).
double periapsisRadius(const glm::dvec3& r, const glm::dvec3& v, double mu);

// Periapsis ALTITUDE (km above body surface) — NaN when not captured.
inline double periapsisAltitude(const glm::dvec3& r, const glm::dvec3& v,
                                double mu, double bodyRadius)
{
    const double rPe = periapsisRadius(r, v, mu);
    return rPe - bodyRadius;   // NaN propagates cleanly
}

} // namespace spacecraft
