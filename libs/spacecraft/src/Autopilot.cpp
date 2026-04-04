#include "apeiron/spacecraft/Autopilot.h"

#include <glm/glm.hpp>

namespace spacecraft {

Wrench Autopilot::compute(const glm::dvec3& omega_body) const
{
    Wrench w{};
    if (mode == AutopilotMode::Off) return w;

    const double omegaMag = glm::length(omega_body);
    if (omegaMag < deadband) return w;

    // Killrot — bang-bang: request maximum counter-torque opposing ω.
    // The magnitude deliberately exceeds RCS authority so the allocator
    // saturates, giving full-on thruster firings instead of small fractions
    // that disappear inside the PWM cycle.
    glm::dvec3 tau = -(maxTorqueNm / omegaMag) * omega_body;
    w[3] = tau.x;
    w[4] = tau.y;
    w[5] = tau.z;
    return w;
}

} // namespace spacecraft
