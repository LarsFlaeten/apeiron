#include "apeiron/spacecraft/OrbitLoader.h"

#include <toml++/toml.hpp>
#include <glm/glm.hpp>
#include <cmath>
#include <stdexcept>

namespace spacecraft {

static constexpr double kDeg    = M_PI / 180.0;
static constexpr double kGM_Earth = 398600.4418; // km³/s²
// Earth mean obliquity J2000: 23.4393°
static constexpr double kObliquity = 23.4393 * kDeg;

OrbitConfig loadOrbitConfig(const std::filesystem::path& tomlPath)
{
    const auto doc = toml::parse_file(tomlPath.string());

    OrbitConfig cfg;

    // [vehicle]
    if (auto* v = doc["vehicle"].as_table()) {
        cfg.name    = (*v)["name"].value_or(std::string{});
        cfg.massKg  = (*v)["mass_kg"].value_or(1.0);
        cfg.glbPath = (*v)["glb"].value_or(std::string{});

        if (auto* it = (*v)["inertia_tensor"].as_array()) {
            if (it->size() == 3) {
                cfg.inertiaDiag = {
                    (*it)[0].value_or(1.0),
                    (*it)[1].value_or(1.0),
                    (*it)[2].value_or(1.0)
                };
            }
        }
    }

    // [orbit]
    const auto* orb = doc["orbit"].as_table();
    if (!orb)
        throw std::runtime_error("Missing [orbit] section in " + tomlPath.string());

    const std::string epochStr = (*orb)["epoch"].value_or(std::string{});
    if (epochStr.empty())
        throw std::runtime_error("Missing orbit.epoch in " + tomlPath.string());

    const double inc     = (*orb)["inclination"].value_or(0.0)  * kDeg;
    const double raan    = (*orb)["raan"].value_or(0.0)          * kDeg;
    const double e       = (*orb)["eccentricity"].value_or(0.0);
    const double argpe   = (*orb)["arg_perigee"].value_or(0.0)   * kDeg;
    const double M0      = (*orb)["mean_anomaly"].value_or(0.0)  * kDeg;
    const double n_revd  = (*orb)["mean_motion"].value_or(0.0);   // rev/day

    const double n_rads  = n_revd * 2.0 * M_PI / 86400.0;        // rad/s
    const double a       = std::cbrt(kGM_Earth / (n_rads * n_rads)); // km
    const double h       = std::sqrt(kGM_Earth * a * (1.0 - e * e));

    astro::OrbitElements& oe = cfg.elements;
    oe.h     = h;
    oe.i     = inc;
    oe.omega = raan;
    oe.e     = e;
    oe.w     = argpe;
    oe.M0    = M0;
    oe.epoch = astro::EphemerisTime::fromString(epochStr);
    oe.mu    = kGM_Earth;
    oe.computeDerivedQuantities();

    return cfg;
}

astro::PosState orbitStateAtEt(const astro::OrbitElements& oe,
                                const astro::EphemerisTime& et)
{
    // toStateVector is non-const in the astro library — work on a copy.
    astro::OrbitElements oeCopy = oe;
    astro::PosState s = oeCopy.toStateVector(et);

    // Rotate J2000 equatorial → ECLIPJ2000 by Earth's obliquity (Rx).
    // astro::Vec3 is glm::dvec3, so this is direct.
    const double ce = std::cos(kObliquity), se = std::sin(kObliquity);
    auto rx = [&](const astro::Vec3& v) -> astro::Vec3 {
        return { v.x,  ce*v.y + se*v.z, -se*v.y + ce*v.z };
    };

    astro::PosState out;
    out.r = rx(s.r);
    out.v = rx(s.v);
    return out;
}

} // namespace spacecraft
