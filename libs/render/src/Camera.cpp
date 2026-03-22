#include "apeiron/render/Camera.h"

#include <glm/gtc/matrix_transform.hpp>

namespace apeiron::render {

Camera::Camera(glm::vec3 position,
               glm::vec3 target,
               glm::vec3 up,
               float     fovDeg,
               float     aspect,
               float     nearPlane,
               float     farPlane)
    : m_position(position)
    , m_target  (target)
    , m_up      (up)
    , m_fovDeg  (fovDeg)
    , m_aspect  (aspect)
    , m_near    (nearPlane)
    , m_far     (farPlane)
{}

glm::mat4 Camera::viewMatrix() const
{
    return glm::lookAt(m_position, m_target, m_up);
}

glm::mat4 Camera::projectionMatrix() const
{
    glm::mat4 proj = glm::perspective(glm::radians(m_fovDeg), m_aspect, m_near, m_far);
    // Vulkan clip space has Y pointing down; GLM was designed for OpenGL (Y up).
    // Flipping the Y scale of the projection matrix corrects the orientation.
    proj[1][1] *= -1.0f;
    return proj;
}

} // namespace apeiron::render
