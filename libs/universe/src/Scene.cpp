#include "apeiron/universe/Scene.h"
#include "apeiron/universe/SceneNode.h"
#include "apeiron/universe/CelestialBody.h"

namespace apeiron::universe {

SceneNode& Scene::addNode(std::unique_ptr<SceneNode> node)
{
    SceneNode& ref = *node;
    m_nodes.push_back(std::move(node));
    return ref;
}

CelestialBody& Scene::addBody(std::string name, std::string naifName,
                              double radiusKm, std::string observer)
{
    auto body = std::make_unique<CelestialBody>(
        std::move(name), std::move(naifName), radiusKm, std::move(observer));
    CelestialBody& ref = *body;
    m_nodes.push_back(std::move(body));
    return ref;
}

void Scene::update(const astro::EphemerisTime& et, const glm::dvec3& cameraWorldPos)
{
    m_origin.recenter(cameraWorldPos);
    for (auto& node : m_nodes)
        node->update(et);
}

const FloatingOrigin&                          Scene::origin() const { return m_origin; }
FloatingOrigin&                                Scene::origin()       { return m_origin; }
const std::vector<std::unique_ptr<SceneNode>>& Scene::nodes()  const { return m_nodes; }

} // namespace apeiron::universe
