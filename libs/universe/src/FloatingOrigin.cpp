#include "apeiron/universe/FloatingOrigin.h"

namespace apeiron::universe {

void FloatingOrigin::recenter(const glm::dvec3& cameraWorldPos)
{
    m_position = cameraWorldPos;
}

glm::vec3 FloatingOrigin::toRenderSpace(const glm::dvec3& worldPos) const
{
    // Subtraction in double eliminates catastrophic cancellation;
    // the resulting offset is small and safe to cast to float.
    return glm::vec3(worldPos - m_position);
}

} // namespace apeiron::universe
