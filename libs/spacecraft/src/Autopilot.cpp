#include "apeiron/spacecraft/Autopilot.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace spacecraft {

Wrench Autopilot::compute(const glm::dquat& currentAttitude,
                          const glm::dvec3& omega_body,
                          const glm::dvec3& inertiaDiag,
                          double            dt,
                          bool&             settleClamp)
{
    settleClamp = false;
    inLargeSlew = false;
    Wrench w{};

    nullVDone = false;

    if (mode == AutopilotMode::Off) {
        m_timedBurnActive = false;
        m_burnTimer       = 0.0;
        return w;
    }

    // =========================================================
    // NullV — null relative velocity to a target body.
    // relVelBody (m/s, body frame) is set by the caller each frame.
    // Outputs a body-frame translational force in Wrench[0..2].
    // Rotation is not touched (caller can combine with Killrot via
    // separate wrench, or let the pilot handle attitude).
    // =========================================================
    if (mode == AutopilotMode::NullV) {
        const double vMag = glm::length(relVelBody);
        if (vMag < nullVDoneThr) {
            nullVDone = true;
            return w;
        }

        // Saturating translation demand — all positive-contributing thrusters
        // fire at full throttle (bang-bang).  Must match the kSat used in
        // simulateRcsForce so that the NavConsole D-NV estimate is consistent
        // with what is actually achieved.
        constexpr double kSat = 1.0e6;
        glm::dvec3 fDir = -relVelBody / vMag;
        w[0] = fDir.x * kSat;
        w[1] = fDir.y * kSat;
        w[2] = fDir.z * kSat;

        if (secondaryMode == AutopilotMode::Off) {
            // No attitude mode combined — damp angular velocity with simple bang-bang.
            const double omegaMag = glm::length(omega_body);
            if (omegaMag > deadband) {
                glm::dvec3 tau = -(maxTorqueNm / omegaMag) * omega_body;
                w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
            }
            return w;
        }
        // secondaryMode is RelVelPlus or RelVelMinus — fall through to attitude
        // hold below, which will fill [3..5] from targetAttitude.
    }

    // =========================================================
    // Killrot
    // =========================================================
    if (mode == AutopilotMode::Killrot) {
        const double omegaMag = glm::length(omega_body);

        if (m_timedBurnActive) {
            if (m_burnTimer > 0.0) {
                m_burnTimer -= dt;
                glm::dvec3 tau = maxTorqueNm * m_burnTorqueDir;
                w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
                return w;
            }
            m_timedBurnActive = false;
        }

        if (omegaMag < settleClampThreshold) {
            if (omegaMag > deadband) settleClamp = true;
            return w;
        }

        if (omegaMag > timedBurnThreshold) {
            inLargeSlew = true;
            glm::dvec3 tau = -(maxTorqueNm / omegaMag) * omega_body;
            w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
            return w;
        }

        // Timed burn
        glm::dvec3 omegaDir = omega_body / omegaMag;
        glm::dvec3 od2      = omegaDir * omegaDir;
        double     I_eff    = od2.x * inertiaDiag.x
                            + od2.y * inertiaDiag.y
                            + od2.z * inertiaDiag.z;
        m_burnTimer       = I_eff * omegaMag / rcsAuthorityNm;
        m_burnTorqueDir   = -omegaDir;
        m_timedBurnActive = true;
        glm::dvec3 tau    = maxTorqueNm * m_burnTorqueDir;
        w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
        m_burnTimer -= dt;
        return w;
    }

    // =========================================================
    // Attitude hold (Prograde / Retrograde)
    // =========================================================

    // Attitude error: rotation from current to target, expressed in body frame.
    // q_err.axis points in the direction we need to rotate, q_err.angle is how far.
    glm::dquat q_err = glm::conjugate(currentAttitude) * targetAttitude;
    if (q_err.w < 0.0) q_err = -q_err;  // shortest arc

    const double half_angle  = std::acos(std::clamp(static_cast<double>(q_err.w), -1.0, 1.0));
    const double error_angle = 2.0 * half_angle;
    const glm::dvec3 error_axis = (error_angle > 1e-6)
        ? glm::dvec3(q_err.x, q_err.y, q_err.z) / std::sin(half_angle)
        : glm::dvec3(1.0, 0.0, 0.0);

    // Rate error in body frame.
    const glm::dvec3 omega_err = omega_body - omegaFF;

    // Rate component along the error axis (signed — positive = moving toward target).
    const double omega_along = glm::dot(omega_err, error_axis);

    // Effective inertia along error axis (diagonal tensor).
    const glm::dvec3 ea2 = error_axis * error_axis;
    const double I_eff = ea2.x * inertiaDiag.x
                       + ea2.y * inertiaDiag.y
                       + ea2.z * inertiaDiag.z;

    // Max angular acceleration available.
    const double alpha = rcsAuthorityNm / std::max(I_eff, 1.0);

    // Parabolic switch value: positive → accelerate toward target,
    // negative → brake.  omega_along * |omega_along| / (2α) is the
    // signed braking distance.
    const double s = error_angle
                   - omega_along * std::abs(omega_along) / (2.0 * alpha);

    // Rate components perpendicular to error axis — damp these independently.
    const glm::dvec3 omega_perp = omega_err - omega_along * error_axis;

    const double omegaErrMag = glm::length(omega_err);

    // Coast band — two independent conditions, both must be satisfied:
    //   1. Attitude error small enough that one pulse would overshoot.
    //   2. Rate error below the fire threshold.
    // Also coast if the parabolic switch value is small (near the switch curve)
    // and attitude error is already inside the threshold — prevents limit cycling
    // when tiny rate noise keeps flipping the switch sign near settled state.
    const bool attOk  = error_angle  < attFireThreshold;
    const bool rateOk = omegaErrMag  < rateFireThreshold;
    const bool switchNearZero = std::abs(s) < attFireThreshold;

    if ((attOk && rateOk) || (attOk && switchNearZero)) {
        if (omegaErrMag < settleClampThreshold && omegaErrMag > deadband)
            settleClamp = true;
        return w;
    }

    // Torque direction: parabolic switch along error axis + cross-axis damping.
    glm::dvec3 tau_dir = std::copysign(1.0, s) * error_axis
                       - KdAtt * omega_perp;
    const double tMag = glm::length(tau_dir);
    if (tMag < 1e-9) return w;

    // Bang-bang: saturate allocator.
    // Flag as large slew only when well outside the fine-correction band.
    inLargeSlew = (error_angle > 10.0 * attFireThreshold);
    glm::dvec3 tau = (maxTorqueNm / tMag) * tau_dir;
    w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
    return w;
}

} // namespace spacecraft
