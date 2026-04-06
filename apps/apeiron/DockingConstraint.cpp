#include "DockingConstraint.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

glm::dvec3 DockingConstraint::portWorldPos(const Spacecraft& sc, const DockPort& p)
{
    // posM is in metres (body frame); convert to km and rotate to inertial.
    return sc.position() + sc.attitude() * glm::dvec3(p.posM) * 1.0e-3;
}

// ---------------------------------------------------------------------------
void DockingConstraint::arm(Spacecraft* active,  DockPort activePort,
                             Spacecraft* passive, DockPort passivePort,
                             const Params& params)
{
    m_active      = active;
    m_passive     = passive;
    m_activePort  = activePort;
    m_passivePort = passivePort;
    m_params      = params;
    m_phase       = Phase::Armed;
    m_holdTimer   = 0.0f;
    m_rangeM      = 0.0f;
    m_attErrDeg   = 0.0f;
    m_closureMs   = 0.0f;
}

void DockingConstraint::disarm()
{
    m_active  = nullptr;
    m_passive = nullptr;
    m_phase   = Phase::Idle;
    m_holdTimer = 0.0f;
}

// ---------------------------------------------------------------------------
void DockingConstraint::update(double dt_s)
{
    if (m_phase == Phase::Idle || !m_active || !m_passive) return;

    if (m_phase == Phase::HardCapture) {
        enforceHardCapture();
        return;
    }

    // ---- Port world-space geometry ----
    const glm::dquat pAtt = m_active->attitude();
    const glm::dquat tAtt = m_passive->attitude();

    const glm::dvec3 pPortPos = portWorldPos(*m_active,  m_activePort);
    const glm::dvec3 tPortPos = portWorldPos(*m_passive, m_passivePort);

    // Port axes in inertial frame.
    const glm::dvec3 pAxisX = pAtt * glm::dvec3(m_activePort.axisX);   // outward (probe)
    const glm::dvec3 pAxisZ = pAtt * glm::dvec3(m_activePort.axisZ);   // roll ref
    const glm::dvec3 tAxisX = tAtt * glm::dvec3(m_passivePort.axisX);  // outward (drogue)
    const glm::dvec3 tAxisZ = tAtt * glm::dvec3(m_passivePort.axisZ);

    // ---- Separation and relative velocity at ports ----
    // sep_m: target port minus active port (positive = target is "ahead")
    const glm::dvec3 sep_m  = (tPortPos - pPortPos) * 1000.0; // km → m
    m_rangeM = static_cast<float>(glm::length(sep_m));

    // Port velocity = CoM velocity + ω × offset (all in inertial, m/s)
    const glm::dvec3 pOffset_m = (pPortPos - m_active->position())  * 1000.0;
    const glm::dvec3 tOffset_m = (tPortPos - m_passive->position()) * 1000.0;
    const glm::dvec3 pPortVel  = m_active->velocity()  * 1000.0
                                + glm::cross(m_active->angularVelocity(),  pOffset_m);
    const glm::dvec3 tPortVel  = m_passive->velocity() * 1000.0
                                + glm::cross(m_passive->angularVelocity(), tOffset_m);
    const glm::dvec3 relVel_ms = tPortVel - pPortVel;

    // Axial (along active port outward axis) and lateral components.
    m_closureMs = static_cast<float>(glm::dot(relVel_ms, pAxisX));

    // ---- Attitude error ----
    // Perfect alignment: pAxisX = -tAxisX, pAxisZ = tAxisZ.
    // Use cross-product magnitudes as linearised rotation errors.
    const glm::dvec3 errX = glm::cross(pAxisX, -tAxisX); // 0 when pAxisX == -tAxisX
    const glm::dvec3 errZ = glm::cross(pAxisZ,  tAxisZ); // 0 when pAxisZ ==  tAxisZ
    // Combined attitude error angle (°)
    const double attErrRad = std::asin(std::clamp(
        (glm::length(errX) + glm::length(errZ)) * 0.5, 0.0, 1.0));
    m_attErrDeg = static_cast<float>(glm::degrees(attErrRad));

    // Relative angular rate magnitude (°/s)
    const glm::dvec3 relOmega = m_passive->angularVelocity() - m_active->angularVelocity();
    const float relOmegaDeg = static_cast<float>(
        glm::degrees(glm::length(relOmega)));

    // ---- Soft-capture detection (Armed phase only) ----
    if (m_phase == Phase::Armed) {
        const float lateralMs = static_cast<float>(
            glm::length(relVel_ms - static_cast<double>(m_closureMs) * pAxisX));

        const bool conditions =
            m_rangeM      < m_params.maxRangeM    &&
            m_closureMs   < m_params.maxClosureMs &&
            lateralMs     < m_params.maxLateralMs &&
            m_attErrDeg   < m_params.maxAttErrDeg &&
            relOmegaDeg   < m_params.maxOmegaDegs;

        if (conditions) {
            m_holdTimer += static_cast<float>(dt_s);
            if (m_holdTimer >= m_params.holdTimeSec)
                m_phase = Phase::SoftCapture;
        } else {
            m_holdTimer = 0.0f;
        }
        return;  // no forces yet in Armed phase
    }

    // ---- SoftCapture: spring-damper force ----
    applySoftCaptureForces();
}

// ---------------------------------------------------------------------------
void DockingConstraint::applySoftCaptureForces()
{
    const glm::dquat pAtt    = m_active->attitude();
    const glm::dquat tAtt    = m_passive->attitude();
    const glm::dquat pAttInv = glm::conjugate(pAtt);
    const glm::dquat tAttInv = glm::conjugate(tAtt);

    const glm::dvec3 pPortPos = portWorldPos(*m_active,  m_activePort);
    const glm::dvec3 tPortPos = portWorldPos(*m_passive, m_passivePort);

    const glm::dvec3 pAxisX = pAtt * glm::dvec3(m_activePort.axisX);
    const glm::dvec3 pAxisZ = pAtt * glm::dvec3(m_activePort.axisZ);
    const glm::dvec3 tAxisX = tAtt * glm::dvec3(m_passivePort.axisX);
    const glm::dvec3 tAxisZ = tAtt * glm::dvec3(m_passivePort.axisZ);

    // Translational spring-damper ----------------------------------------
    const glm::dvec3 sep_m    = (tPortPos - pPortPos) * 1000.0;

    const glm::dvec3 pOffset_m = (pPortPos - m_active->position())  * 1000.0;
    const glm::dvec3 tOffset_m = (tPortPos - m_passive->position()) * 1000.0;
    const glm::dvec3 pPortVel  = m_active->velocity()  * 1000.0
                                + glm::cross(m_active->angularVelocity(),  pOffset_m);
    const glm::dvec3 tPortVel  = m_passive->velocity() * 1000.0
                                + glm::cross(m_passive->angularVelocity(), tOffset_m);
    const glm::dvec3 relVel_ms = tPortVel - pPortVel;

    // Spring force in inertial frame (N):  F = k·Δx + c·Δv
    const glm::dvec3 F_inertial = m_params.kTrans * sep_m
                                 + m_params.cTrans * relVel_ms;

    // Convert force to body frame and apply (Newton's 3rd law on passive).
    const glm::dvec3 F_active_body  =  pAttInv * F_inertial;
    const glm::dvec3 F_passive_body = -(tAttInv * F_inertial);

    m_active ->addBodyForce(F_active_body);
    m_passive->addBodyForce(F_passive_body);

    // Torque from offset (r × F, in body frame) ---------------------------
    const glm::dvec3 pOff_body = pAttInv * pOffset_m;
    const glm::dvec3 tOff_body = tAttInv * tOffset_m;

    m_active ->addBodyTorque(glm::cross(pOff_body, F_active_body));
    m_passive->addBodyTorque(glm::cross(tOff_body, F_passive_body));

    // Rotational spring-damper -------------------------------------------
    // Drive attitude error to zero:
    //   errX = pAxisX × (-tAxisX) → 0 when probes face each other
    //   errZ = pAxisZ × tAxisZ    → 0 when roll refs aligned
    const glm::dvec3 errX = glm::cross(pAxisX, -tAxisX);
    const glm::dvec3 errZ = glm::cross(pAxisZ,  tAxisZ);
    const glm::dvec3 attErrWorld = errX + errZ;

    const glm::dvec3 relOmega = m_passive->angularVelocity()
                               - m_active->angularVelocity();

    // Rotational spring-damper torque (inertial frame, N·m)
    const glm::dvec3 T_inertial = m_params.kRot * attErrWorld
                                 + m_params.cRot * relOmega;

    m_active ->addBodyTorque( pAttInv * T_inertial);
    m_passive->addBodyTorque(-(tAttInv * T_inertial));
}

// ---------------------------------------------------------------------------
void DockingConstraint::initiateHardCapture()
{
    if (m_phase != Phase::SoftCapture || !m_active || !m_passive) return;

    const glm::dquat tAtt    = m_passive->attitude();
    const glm::dquat tAttInv = glm::conjugate(tAtt);

    // Store active CoM position in passive body frame (km)
    m_hardRelPos = tAttInv * (m_active->position() - m_passive->position());

    // Store active attitude relative to passive
    m_hardRelAtt = tAttInv * m_active->attitude();

    // Mark active as a child so the main loop skips its own integration
    // (index 0 = Orion, index 1 = ISS — caller should ensure correct indices,
    //  but the parent-tracking is done here for convenience; the index used
    //  here is the passive's position in the spacecraft vector, which we
    //  approximate as 1 since ISS is always index 1 in the current setup.
    //  A cleaner future refactor would pass spacecraft indices to arm().)
    m_active->setParent(1);

    m_phase = Phase::HardCapture;
}

// ---------------------------------------------------------------------------
void DockingConstraint::enforceHardCapture()
{
    if (!m_active || !m_passive) return;

    const glm::dquat tAtt     = m_passive->attitude();
    const glm::dvec3 tPos     = m_passive->position();
    const glm::dvec3 tVel     = m_passive->velocity();
    const glm::dvec3 tOmega   = m_passive->angularVelocity();

    // Active position: passive CoM + rotated relative offset
    const glm::dvec3 offset_inertial = tAtt * m_hardRelPos; // km
    const glm::dvec3 newPos = tPos + offset_inertial;
    const glm::dquat newAtt = tAtt * m_hardRelAtt;

    // Active velocity: v_CoM + ω × r (km/s)
    const glm::dvec3 newVel = tVel + glm::cross(tOmega, offset_inertial);

    m_active->setPosition(newPos);
    m_active->setVelocity(newVel);
    m_active->setAttitude(newAtt);
    m_active->setAngularVelocity(tOmega);
}
