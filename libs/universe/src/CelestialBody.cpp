#include "apeiron/universe/CelestialBody.h"
#include "apeiron/universe/Ephemeris.h"

namespace apeiron::universe {

CelestialBody::CelestialBody(std::string name, std::string naifName,
                              double radiusKm, std::string observer,
                              std::string frame)
    : SceneNode(std::move(name))
    , m_naifName (std::move(naifName))
    , m_observer (std::move(observer))
    , m_frame    (std::move(frame))
    , m_radiusKm (radiusKm)
{}

const std::string& CelestialBody::naifName() const { return m_naifName; }
const std::string& CelestialBody::observer() const { return m_observer; }
double             CelestialBody::radiusKm()  const { return m_radiusKm; }

void CelestialBody::update(const astro::EphemerisTime& et)
{
    auto state = getState(m_naifName, et, m_observer, m_frame);
    setPosition(state.position);
}

} // namespace apeiron::universe
