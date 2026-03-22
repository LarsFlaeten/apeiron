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
                  double      radiusKm,
                  std::string observer = "SOLAR SYSTEM BARYCENTER",
                  std::string frame    = "ECLIPJ2000");

    const std::string& naifName()  const;
    const std::string& observer()  const;
    double             radiusKm()  const;

    // Queries SPICE and updates worldPosition().
    void update(const astro::EphemerisTime& et) override;

private:
    std::string m_naifName;
    std::string m_observer;
    std::string m_frame;
    double      m_radiusKm = 0.0;
};

} // namespace apeiron::universe
