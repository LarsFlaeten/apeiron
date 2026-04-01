#pragma once

#include "Thruster.h"

#include <glm/glm.hpp>
#include <vector>
#include <array>

namespace spacecraft {

// 6-component wrench: [Fx, Fy, Fz, Tx, Ty, Tz] in model-space SI units.
using Wrench = std::array<double, 6>;

// ---------------------------------------------------------------------------
// SpacecraftModel
//
// Holds the physical description of a spacecraft loaded from a TOML manifest.
// Builds the 6×N control effectiveness matrix at load time and provides a
// pseudoinverse solver to allocate throttles from a desired wrench.
//
// Coordinate convention matches the manifest (model space):
//   typically +X = aft, +Y = up, +Z = starboard for Orion,
//   but the matrix is built purely from the thruster vectors so any
//   consistent convention works.
// ---------------------------------------------------------------------------
class SpacecraftModel {
public:
    float         massKg       = 1.0f;
    glm::vec3     centerOfMass {};      // model space, metres
    glm::vec3     inertiaDiag  {};      // principal moments, kg·m²

    std::vector<Thruster> thrusters;

    // Build the 6×N effectiveness matrix and precompute its pseudoinverse.
    // Must be called once after all thrusters are loaded.
    void buildEffectivenessMatrix();

    // Solve for per-thruster throttles given a desired wrench.
    // Returns throttles clamped to [0, 1]; resultant wrench may be
    // scaled down if the desired wrench exceeds actuator capacity.
    void solveAllocation(const Wrench& desired);

    // Apply the current throttle values as body force + torque.
    // Outputs are in model space, SI units (N and N·m).
    void accumulateWrench(glm::vec3& forceOut, glm::vec3& torqueOut) const;

private:
    // Stored as a flat column-major array: B[row + 6*col].
    // 6 rows (Fx Fy Fz Tx Ty Tz), N columns (one per thruster).
    std::vector<double> m_B;       // effectiveness matrix
    std::vector<double> m_Bpinv;   // pseudoinverse (N×6, column-major)
    int                 m_N = 0;
};

} // namespace spacecraft
