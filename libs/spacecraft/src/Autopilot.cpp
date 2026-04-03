#include "apeiron/spacecraft/Autopilot.h"

#include <glm/glm.hpp>

namespace spacecraft {

Wrench Autopilot::compute(const glm::dvec3& omega_body)
{
    Wrench w{};
    if (mode == AutopilotMode::Off) return w;

    if (glm::length(omega_body) < deadband) {
        // Settled — disengage so the pilot gets a clear "done" signal.
        mode = AutopilotMode::Off;
        return w;
    }

    // Killrot: pure damping torque = -Kd × ω_body
    glm::dvec3 tau = -kdRot * omega_body;
    w[3] = tau.x;
    w[4] = tau.y;
    w[5] = tau.z;
    return w;
}

} // namespace spacecraft
