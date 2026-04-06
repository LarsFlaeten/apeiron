#pragma once

#include "Spacecraft.h"
#include "apeiron/render/GltfModel.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// ---------------------------------------------------------------------------
// DockingConstraintParams — tuning parameters (SI units throughout).
// Declared outside the class so it can be used as a default argument to arm().
// ---------------------------------------------------------------------------
struct DockingConstraintParams {
    // ---- Soft-capture trigger thresholds ----
    float maxRangeM    = 0.15f;  // m   — port-to-port distance
    float maxClosureMs = 0.03f;  // m/s — axial approach speed
    float maxLateralMs = 0.01f;  // m/s — lateral speed
    float maxAttErrDeg = 5.0f;   // °   — combined attitude error
    float maxOmegaDegs = 0.5f;   // °/s — relative angular rate
    float holdTimeSec  = 0.5f;   // s   — must hold conditions this long

    // ---- Spring-damper (soft capture) ----
    double kTrans = 150'000.0;  // N/m
    double cTrans = 100'000.0;  // N·s/m
    double kRot   = 400'000.0;  // N·m/rad
    double cRot   = 250'000.0;  // N·m·s/rad

    // Hard capture only allowed within this range (m); prevents accidental lock at distance.
    float maxHardCaptureM   = 0.5f;
    // If soft-capture range diverges beyond this, auto-release back to Armed (m).
    float softCaptureBreakM = 2.0f;
};

// ---------------------------------------------------------------------------
// DockingConstraint
//
// Physics constraint that models the probe/drogue soft-capture and hard-capture
// sequence between two spacecraft.
//
// Phase state machine:
//   Idle        — not active; no forces applied.
//   Armed       — monitoring conditions; no forces yet.
//   SoftCapture — spring-damper engaged; probe is latched in the drogue.
//                 Transition is automatic when all capture conditions are met.
//   HardCapture — rigid lock; active body follows passive body exactly.
//                 Transition is user-initiated via initiateHardCapture().
//
// Call arm() once when docking mode is entered with a valid port selection,
// disarm() when the mode or port selection changes.
//
// Inside the physics sub-step loop, call update(dt) BEFORE Spacecraft::update()
// so that constraint forces are included in this tick's integration.
// ---------------------------------------------------------------------------
class DockingConstraint
{
public:
    using DockPort = apeiron::render::GltfModel::DockingPort;
    using Params   = DockingConstraintParams;

    enum class Phase { Idle, Armed, SoftCapture, HardCapture };

    // Arm the constraint.  active = probe (Orion), passive = drogue (ISS).
    // Ports must already be transformed into spacecraft body frame (rollFix applied).
    void arm(Spacecraft* active,  DockPort activePort,
             Spacecraft* passive, DockPort passivePort,
             const Params& params = Params{});

    // Disarm and reset to Idle.
    void disarm();

    // Pre-step: apply spring-damper forces and check capture conditions.
    // Call BEFORE Spacecraft::update() so forces are included in this tick.
    void update(double dt_s);

    // Post-step: enforce rigid lock for hard capture.
    // Call AFTER all Spacecraft::update() calls so Orion follows ISS's
    // newly-integrated position (avoids the ~77 m one-tick orbital lag).
    void enforcePostStep();

    // User-initiated hard capture.  Only valid in SoftCapture phase AND when
    // range is within maxHardCaptureM (default 0.5 m).
    void initiateHardCapture();

    // Release from dock (any phase).  Clears parent, returns to Armed if a
    // port is still selected, otherwise Idle.
    void release();

    // ---- Query ----
    Phase phase()     const { return m_phase; }
    float rangeM()    const { return m_rangeM; }    // port-to-port distance (m)
    float attErrDeg() const { return m_attErrDeg; } // combined attitude error (°)
    float closureMs() const { return m_closureMs; } // axial approach speed (m/s)
    float holdTimer() const { return m_holdTimer; } // hold time so far (s)

private:
    static glm::dvec3 portWorldPos(const Spacecraft& sc, const DockPort& p);
    void applySoftCaptureForces();
    void enforceHardCapture();

    // ---- Configuration ----
    Spacecraft* m_active  = nullptr;
    Spacecraft* m_passive = nullptr;
    DockPort    m_activePort{};
    DockPort    m_passivePort{};
    Params      m_params{};

    // ---- State ----
    Phase m_phase     = Phase::Idle;
    float m_holdTimer = 0.0f;

    // Diagnostics (updated each tick in Armed / SoftCapture).
    float m_rangeM    = 0.0f;
    float m_attErrDeg = 0.0f;
    float m_closureMs = 0.0f;

    // ---- Hard-capture stored relative transform ----
    glm::dvec3 m_hardRelPos{0.0};           // active CoM in passive body frame (km)
    glm::dquat m_hardRelAtt{1.0, 0.0, 0.0, 0.0}; // q_active = q_passive * m_hardRelAtt
};
