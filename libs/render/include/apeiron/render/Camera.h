#pragma once

#include <glm/glm.hpp>

namespace apeiron::render {

// Perspective camera.  Produces a view matrix via glm::lookAt and a
// projection matrix via glm::perspective with the Vulkan Y-flip applied.
//
// All positions are in render-space (32-bit float, km relative to the
// floating origin).  The camera will be driven by the Scene's floating
// origin once we connect the render and universe layers.
class Camera {
public:
    Camera(glm::vec3 position,
           glm::vec3 target,
           glm::vec3 up,
           float     fovDeg,
           float     aspect,
           float     nearPlane,
           float     farPlane);

    glm::mat4 viewMatrix()       const;
    glm::mat4 projectionMatrix() const;

    // Convenience: projection * view, ready to multiply by a model matrix.
    glm::mat4 viewProjection() const { return projectionMatrix() * viewMatrix(); }

    void setPosition(const glm::vec3& pos)    { m_position = pos;    }
    void setTarget  (const glm::vec3& target) { m_target   = target; }
    void setAspect  (float aspect)            { m_aspect   = aspect; }

    const glm::vec3& position() const { return m_position; }
    float            near()     const { return m_near;     }
    float            far()      const { return m_far;      }

private:
    glm::vec3 m_position;
    glm::vec3 m_target;
    glm::vec3 m_up;
    float     m_fovDeg;
    float     m_aspect;
    float     m_near;
    float     m_far;
};

} // namespace apeiron::render
