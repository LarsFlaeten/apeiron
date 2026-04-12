#pragma once

#include <glm/glm.hpp>

namespace spacecraft {

// ---------------------------------------------------------------------------
// Lambert solver — Izzo's method (Dario Izzo, ESA, 2015).
//
// Solves the two-point boundary value problem: find the velocity vectors v1
// and v2 such that an unperturbed Keplerian arc connects r1 to r2 in time tof
// under the gravitational parameter mu.
//
// Reference:
//   Izzo, D., "Revisiting Lambert's problem", Celestial Mechanics and
//   Dynamical Astronomy, 121(1), 2015.  https://doi.org/10.1007/s10569-014-9587-y
//
// Parameters:
//   mu      — gravitational parameter of the central body (km³/s²)
//   r1, r2  — position vectors at departure and arrival (km), same inertial frame
//   tof     — time of flight (seconds, must be > 0)
//   prograde— true for prograde transfer (short-way), false for retrograde
//   v1[out] — velocity at departure (km/s)
//   v2[out] — velocity at arrival (km/s)
//
// Returns true on convergence, false if the problem is degenerate or does not
// converge within the iteration limit (caller should discard the result).
// ---------------------------------------------------------------------------
bool solveLambert(double           mu,
                  const glm::dvec3& r1,
                  const glm::dvec3& r2,
                  double            tof,
                  bool              prograde,
                  glm::dvec3&       v1,
                  glm::dvec3&       v2);

} // namespace spacecraft
