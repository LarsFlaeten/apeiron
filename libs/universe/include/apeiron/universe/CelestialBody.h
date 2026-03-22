#pragma once

#include "apeiron/universe/SceneNode.h"

#include <string>

namespace apeiron::universe {

// A SceneNode whose position is driven by SPICE ephemeris each update.
//
// naifName is the NAIF body name (e.g. "EARTH", "MARS BARYCENTER").
// observer is the reference body for getState(); defaults to SSB so
// worldPosition() is always in the SSB-centred J2000 frame.
class CelestialBody : public SceneNode {
public:
    CelestialBody(std::string name,
                  std::string naifName,
                  std::string observer = "SOLAR SYSTEM BARYCENTER");

    const std::string& naifName() const;
    const std::string& observer() const;

    // Queries SPICE and updates worldPosition().
    void update(const astro::EphemerisTime& et) override;

private:
    std::string m_naifName;
    std::string m_observer;
};

} // namespace apeiron::universe
