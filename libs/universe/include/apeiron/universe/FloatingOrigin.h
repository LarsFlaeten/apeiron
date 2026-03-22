#pragma once

#include <glm/glm.hpp>

namespace apeiron::universe {

// Manages a camera-anchored reference point for render-space conversion.
//
// The simulator works in 64-bit double precision (km from SSB). The GPU works
// in 32-bit float. Naively casting a large world coordinate to float loses
// precision. The fix: subtract the origin in double first, then cast the small
// offset to float. This class owns that subtraction.
//
// Usage each frame:
//   origin.recenter(cameraWorldPos);        // move anchor to camera
//   glm::vec3 p = origin.toRenderSpace(bodyWorldPos);  // safe float offset
class FloatingOrigin {
public:
    // Move the anchor to the camera's current world position.
    void recenter(const glm::dvec3& cameraWorldPos);

    // Convert a 64-bit world position (km from SSB) to a 32-bit render offset
    // (km relative to this origin). Subtraction is done in double.
    glm::vec3 toRenderSpace(const glm::dvec3& worldPos) const;

    const glm::dvec3& position() const { return m_position; }

private:
    glm::dvec3 m_position{0.0, 0.0, 0.0};
};

} // namespace apeiron::universe
