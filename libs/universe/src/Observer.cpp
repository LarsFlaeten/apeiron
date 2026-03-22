#include "apeiron/universe/Observer.h"
#include "apeiron/universe/Ephemeris.h"

namespace apeiron::universe {

Observer::Observer(std::string body, std::string target, std::string frame)
    : m_body  (std::move(body))
    , m_target(std::move(target))
    , m_frame (std::move(frame))
{}

glm::dvec3 Observer::worldPosition(const astro::EphemerisTime& et) const
{
    return getState(m_body, et, "SOLAR SYSTEM BARYCENTER", m_frame).position;
}

glm::dvec3 Observer::targetPosition(const astro::EphemerisTime& et) const
{
    return getState(m_target, et, "SOLAR SYSTEM BARYCENTER", m_frame).position;
}

} // namespace apeiron::universe
