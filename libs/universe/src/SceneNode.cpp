#include "apeiron/universe/SceneNode.h"
#include "apeiron/universe/FloatingOrigin.h"

namespace apeiron::universe {

SceneNode::SceneNode(std::string name)
    : m_name(std::move(name))
{}

const std::string& SceneNode::name()          const { return m_name; }
const glm::dvec3&  SceneNode::worldPosition() const { return m_worldPosition; }

void SceneNode::setPosition(const glm::dvec3& pos) { m_worldPosition = pos; }

glm::vec3 SceneNode::renderPosition(const FloatingOrigin& origin) const
{
    return origin.toRenderSpace(m_worldPosition);
}

void SceneNode::update(const astro::EphemerisTime& /*et*/) {}

} // namespace apeiron::universe
