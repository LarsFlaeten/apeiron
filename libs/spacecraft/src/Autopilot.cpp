#include "apeiron/spacecraft/Autopilot.h"

#include <glm/glm.hpp>

namespace spacecraft {

Wrench Autopilot::compute(const glm::dvec3& omega_body,
                          const glm::dvec3& inertiaDiag,
                          double            dt,
                          bool&             settleClamp)
{
    settleClamp = false;
    Wrench w{};

    if (mode == AutopilotMode::Off) {
        m_timedBurnActive = false;
        m_burnTimer       = 0.0;
        return w;
    }

    const double omegaMag = glm::length(omega_body);

    // --- Continue an active timed burn ---
    if (m_timedBurnActive) {
        if (m_burnTimer > 0.0) {
            m_burnTimer -= dt;
            glm::dvec3 tau = maxTorqueNm * m_burnTorqueDir;
            w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
            return w;
        }
        m_timedBurnActive = false;
        // Fall through to re-evaluate ω.
    }

    // --- Phase 3: settle clamp ---
    // Below the hardware minimum-impulse floor, directly zero angular velocity.
    if (omegaMag < settleClampThreshold) {
        if (omegaMag > deadband)
            settleClamp = true;  // caller will zero ω this frame
        return w;
    }

    // --- Phase 1: bang-bang ---
    if (omegaMag > timedBurnThreshold) {
        glm::dvec3 tau = -(maxTorqueNm / omegaMag) * omega_body;
        w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
        return w;
    }

    // --- Phase 2: timed burn ---
    // Effective inertia along the ω axis (diagonal tensor assumed).
    glm::dvec3 omegaDir = omega_body / omegaMag;
    glm::dvec3 od2      = omegaDir * omegaDir;
    double     I_eff    = od2.x * inertiaDiag.x
                        + od2.y * inertiaDiag.y
                        + od2.z * inertiaDiag.z;

    m_burnTimer       = I_eff * omegaMag / rcsAuthorityNm;
    m_burnTorqueDir   = -omegaDir;
    m_timedBurnActive = true;

    // Fire on the first frame of the burn.
    glm::dvec3 tau = maxTorqueNm * m_burnTorqueDir;
    w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
    m_burnTimer -= dt;
    return w;
}

} // namespace spacecraft
