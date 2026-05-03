#include "apeiron/spacecraft/Autopilot.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace spacecraft {

void Autopilot::updateOrbitalTarget(const glm::dvec3& r, const glm::dvec3& v,
                                    const glm::dquat& att)
{
    using M = AutopilotMode;
    if (mode != M::Prograde   && mode != M::Retrograde &&
        mode != M::NormalPlus && mode != M::NormalMinus)
        return;

    const double vMag = glm::length(v);
    const double hMag = glm::length(glm::cross(r, v));
    if (vMag < 1e-9 || hMag < 1e-6) return;

    const glm::dvec3 T =  glm::normalize(v);                  // prograde
    const glm::dvec3 N =  glm::normalize(glm::cross(r, v));   // orbit normal
    const glm::dvec3 R =  glm::cross(T, N);                   // radial outward

    // dmat3 columns = body +X, +Y, +Z expressed in inertial frame.
    // Prograde:   nose=T,  up=-R (nadir),  right= N
    // Retrograde: nose=-T, up= R (zenith), right= N
    // Normal+:    nose=N,  up= T,          right=-R
    // Normal-:    nose=-N, up=-T,          right=-R
    switch (mode) {
        case M::Prograde:
            targetAttitude = glm::quat_cast(glm::dmat3( T, -R,  N)); break;
        case M::Retrograde:
            targetAttitude = glm::quat_cast(glm::dmat3(-T,  R,  N)); break;
        // Normal+/Normal-: pure pitch from Prograde/Retrograde — no roll.
        // Keep up(+Y) = nadir(-R) identical to Prograde so the only rotation
        // is pitch (nose sweeps T → N while +Y stays nadir).
        //   Normal+: nose=N, up=-R, right=-T   (cross(N,-R) = -(N×R) = -T ✓)
        //   Normal-: nose=-N, up=-R, right=+T   (cross(-N,-R) = N×R = T ✓)
        case M::NormalPlus:
            targetAttitude = glm::quat_cast(glm::dmat3( N, -R, -T)); break;
        case M::NormalMinus:
            targetAttitude = glm::quat_cast(glm::dmat3(-N, -R,  T)); break;
        default: break;
    }

    // Feedforward: orbital angular velocity ω = (r × v) / |r|²
    // Prograde/Retrograde: prograde direction rotates once per orbit → needs ff.
    // NormalPlus/NormalMinus: orbit normal is inertially fixed (no precession) →
    //   zero feedforward; applying it introduces a spurious rate error and oscillation.
    if (mode == M::Prograde || mode == M::Retrograde) {
        glm::dvec3 omegaOrb = glm::cross(r, v) / glm::dot(r, r);
        omegaFF = glm::dvec3(glm::conjugate(att) * omegaOrb);
    } else {
        omegaFF = glm::dvec3(0.0);
    }
}

void Autopilot::updateMccTarget(const glm::dquat& att)
{
    if (mode != AutopilotMode::MccPlus) return;

    const double len = glm::length(mccDirInertial);
    if (len < 1e-9) return;

    const glm::dvec3 T = mccDirInertial / len;

    // Build an orthonormal frame: T is body +X (burn direction).
    // Pick a reference vector to minimise roll during transitions:
    // prefer current +Y projected perpendicular to T.
    glm::dvec3 upRef = att * glm::dvec3(0, 1, 0);
    upRef -= glm::dot(upRef, T) * T;
    if (glm::length(upRef) < 0.1) {
        upRef = att * glm::dvec3(0, 0, 1);
        upRef -= glm::dot(upRef, T) * T;
    }
    if (glm::length(upRef) < 1e-9) {
        // Last resort: arbitrary perpendicular
        upRef = (std::abs(T.z) < 0.9) ? glm::dvec3(0, 0, 1) : glm::dvec3(1, 0, 0);
        upRef -= glm::dot(upRef, T) * T;
    }
    upRef = glm::normalize(upRef);
    const glm::dvec3 right = glm::normalize(glm::cross(T, upRef));
    upRef = glm::normalize(glm::cross(right, T));  // re-orthogonalise

    targetAttitude = glm::quat_cast(glm::dmat3(T, upRef, right));
    omegaFF        = glm::dvec3(0.0);
}

void Autopilot::updateRelVelTarget(const glm::dquat& att)
{
    using M = AutopilotMode;
    const M relVelMode = (mode == M::RelVelPlus || mode == M::RelVelMinus)
                         ? mode : secondaryMode;
    if (relVelMode != M::RelVelPlus && relVelMode != M::RelVelMinus)
        return;

    const glm::dvec3 relVelInertial = att * relVelBody;
    const double rvMag = glm::length(relVelInertial);
    if (rvMag < 1e-3) return;

    glm::dvec3 V = relVelInertial / rvMag;
    if (relVelMode == M::RelVelMinus) V = -V;

    // Up reference: current +Y body projected ⊥ to V (minimises roll during transition).
    glm::dvec3 upRef = att * glm::dvec3(0, 1, 0);
    upRef -= glm::dot(upRef, V) * V;
    if (glm::length(upRef) < 0.1) {
        upRef = att * glm::dvec3(0, 0, 1);
        upRef -= glm::dot(upRef, V) * V;
    }
    upRef = glm::normalize(upRef);
    glm::dvec3 right = glm::normalize(glm::cross(V, upRef));
    upRef = glm::normalize(glm::cross(right, V));  // re-orthogonalise

    targetAttitude = glm::quat_cast(glm::dmat3(V, upRef, right));
    omegaFF        = glm::dvec3(0.0);
}

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
    // Killrot — proportional rate damper, saturated at rcsAuthorityNm.
    //
    // kD = 0.5 * I_eff / dt targets 50% reduction in omega per frame in the
    // unsaturated regime, giving monotonic geometric convergence to zero.
    // When |omega| is large the demand saturates at rcsAuthorityNm (bang-bang).
    // No timed burns, no oscillation from discrete-time overshoot.
    // =========================================================
    if (mode == AutopilotMode::Killrot) {
        m_timedBurnActive = false;
        m_burnTimer       = 0.0;

        const double omegaMag = glm::length(omega_body);

        if (omegaMag < settleClampThreshold) {
            if (omegaMag > deadband) settleClamp = true;
            return w;
        }

        inLargeSlew = (omegaMag > timedBurnThreshold);

        // Effective inertia along current spin axis.
        const glm::dvec3 omegaDir = omega_body / omegaMag;
        const glm::dvec3 od2      = omegaDir * omegaDir;
        const double I_eff = od2.x * inertiaDiag.x
                           + od2.y * inertiaDiag.y
                           + od2.z * inertiaDiag.z;

        // Proportional: halve omega each frame; saturate for large rates.
        const double kD   = 0.5 * std::max(I_eff, 1.0) / std::max(dt, 1e-4);
        glm::dvec3   tau  = -kD * omega_body;
        const double tMag = glm::length(tau);
        if (tMag > rcsAuthorityNm)
            tau *= rcsAuthorityNm / tMag;

        w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
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
    // MUST equal the torque that is actually delivered when bang-bang fires in the
    // braking direction — so that the parabolic braking-distance estimate is accurate.
    // rcsAuthorityNm is calibrated at startup from actual thruster geometry (see main.cpp).
    // Using maxTorqueNm (the *requested* torque) would underestimate the effect if
    // the allocator delivers less than requested, causing it to brake too late → overshoot.
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

    // Torque direction: bang-bang along error axis, blended with cross-axis damping.
    // The combined direction is normalised so the allocator is always saturated,
    // delivering exactly rcsAuthorityNm along whatever direction it can best achieve.
    // alpha = rcsAuthorityNm / I_eff is therefore the correct braking-distance estimate
    // for a pure single-axis slew (omega_perp = 0, |tau_dir| = 1).
    glm::dvec3 tau_dir = std::copysign(1.0, s) * error_axis
                       - KdAtt * omega_perp;
    const double tMag = glm::length(tau_dir);
    if (tMag < 1e-9) return w;

    inLargeSlew = (error_angle > 10.0 * attFireThreshold);
    glm::dvec3 tau = (maxTorqueNm / tMag) * tau_dir;
    w[3] = tau.x; w[4] = tau.y; w[5] = tau.z;
    return w;
}

} // namespace spacecraft
