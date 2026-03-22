#include "apeiron/universe/CelestialBody.h"
#include "apeiron/universe/Ephemeris.h"

namespace apeiron::universe {

CelestialBody::CelestialBody(std::string name, std::string naifName, std::string observer)
    : SceneNode(std::move(name))
    , m_naifName(std::move(naifName))
    , m_observer(std::move(observer))
{}

const std::string& CelestialBody::naifName() const { return m_naifName; }
const std::string& CelestialBody::observer() const { return m_observer; }

void CelestialBody::update(const astro::EphemerisTime& et)
{
    auto state = getState(m_naifName, et, m_observer);
    setPosition(state.position);
}

} // namespace apeiron::universe
