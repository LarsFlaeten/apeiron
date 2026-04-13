#include "apeiron/spacecraft/VehicleConfig.h"

#include <toml++/toml.hpp>
#include <cmath>
#include <stdexcept>

namespace spacecraft {

static constexpr double kDeg       = M_PI / 180.0;
static constexpr double kGM_Earth  = 398600.4418;    // km³/s²
static constexpr double kObliquity = 23.4393 * kDeg; // Earth J2000 mean obliquity

static glm::vec3 tomlVec3f(const toml::array& a, const char* field)
{
    if (a.size() != 3)
        throw std::runtime_error(std::string(field) + " must be an array of 3 values");
    return { static_cast<float>(a[0].value_or(0.0)),
             static_cast<float>(a[1].value_or(0.0)),
             static_cast<float>(a[2].value_or(0.0)) };
}

static glm::dvec3 tomlVec3d(const toml::array& a, const char* field)
{
    if (a.size() != 3)
        throw std::runtime_error(std::string(field) + " must be an array of 3 values");
    return { a[0].value_or(0.0), a[1].value_or(0.0), a[2].value_or(0.0) };
}

static ThrusterType parseType(std::string_view s)
{
    if (s == "main_engine") return ThrusterType::MainEngine;
    if (s == "auxiliary")   return ThrusterType::Auxiliary;
    if (s == "rcs")         return ThrusterType::RCS;
    throw std::runtime_error("Unknown thruster type: " + std::string(s));
}

VehicleConfig loadVehicleConfig(const std::filesystem::path& tomlPath)
{
    const auto doc = toml::parse_file(tomlPath.string());

    // ---- [vehicle] — required ----
    const auto* vt = doc["vehicle"].as_table();
    if (!vt)
        throw std::runtime_error("[vehicle] section missing in " + tomlPath.string());

    VehicleConfig vc;
    vc.name   = (*vt)["name"].value_or(std::string{});
    vc.massKg = static_cast<float>((*vt)["mass_kg"].value_or(1.0));
    vc.glbPath = (*vt)["glb"].value_or(std::string{});

    if (auto* it = (*vt)["inertia_tensor"].as_array())
        vc.inertiaDiag = tomlVec3d(*it, "inertia_tensor");

    if (auto* com = (*vt)["center_of_mass"].as_array())
        vc.centerOfMass = tomlVec3f(*com, "center_of_mass");

    // ---- [orbit] — optional ----
    if (const auto* orb = doc["orbit"].as_table()) {
        const std::string epochStr = (*orb)["epoch"].value_or(std::string{});
        if (epochStr.empty())
            throw std::runtime_error("orbit.epoch missing in " + tomlPath.string());

        const double inc    = (*orb)["inclination"].value_or(0.0) * kDeg;
        const double raan   = (*orb)["raan"].value_or(0.0)        * kDeg;
        const double e      = (*orb)["eccentricity"].value_or(0.0);
        const double argpe  = (*orb)["arg_perigee"].value_or(0.0) * kDeg;
        const double M0     = (*orb)["mean_anomaly"].value_or(0.0) * kDeg;
        const double n_revd = (*orb)["mean_motion"].value_or(0.0);   // rev/day

        const double n_rads = n_revd * 2.0 * M_PI / 86400.0;
        const double a      = std::cbrt(kGM_Earth / (n_rads * n_rads));
        const double h      = std::sqrt(kGM_Earth * a * (1.0 - e * e));

        astro::OrbitElements& oe = vc.orbitElements;
        oe.h     = h;
        oe.i     = inc;
        oe.omega = raan;
        oe.e     = e;
        oe.w     = argpe;
        oe.M0    = M0;
        oe.epoch = astro::EphemerisTime::fromString(epochStr);
        oe.mu    = kGM_Earth;
        oe.computeDerivedQuantities();

        vc.hasOrbit = true;
    }

    // ---- [[thrusters]] — optional ----
    if (const auto* thr_arr = doc["thrusters"].as_array()) {
        for (const auto& entry : *thr_arr) {
            const auto& t = *entry.as_table();
            Thruster thr;
            thr.id       = t["id"].value_or(std::string{});
            thr.type     = parseType(t["type"].value_or(std::string_view{"rcs"}));
            thr.quad     = static_cast<int>(t["quad"].value_or(0LL));
            thr.thrustN  = static_cast<float>(t["thrust_n"].value_or(0.0));
            thr.ispS     = static_cast<float>(t["isp_s"].value_or(0.0));
            thr.refNode      = t["ref_node"].value_or(std::string{});
            thr.exhaustNode  = t["exhaust_node"].value_or(std::string{});
            thr.exhaustScale    = static_cast<float>(t["exhaust_scale"].value_or(1.0));
            thr.plumeIntensity  = static_cast<float>(t["plume_intensity"].value_or(1.0));
            if (auto* col = t["plume_color"].as_array())
                thr.plumeColor = tomlVec3f(*col, "plume_color");
            if (auto* pos = t["position"].as_array())
                thr.position = tomlVec3f(*pos, "position");
            if (auto* dir = t["direction"].as_array())
                thr.direction = glm::normalize(tomlVec3f(*dir, "direction"));
            vc.model.thrusters.push_back(std::move(thr));
        }
        vc.model.massKg       = vc.massKg;
        vc.model.centerOfMass = vc.centerOfMass;
        vc.model.inertiaDiag  = glm::vec3(vc.inertiaDiag);
        vc.model.buildEffectivenessMatrix();
    }

    return vc;
}

astro::PosState vehicleStateAtEt(const VehicleConfig& vc,
                                  const astro::EphemerisTime& et)
{
    astro::OrbitElements oe = vc.orbitElements;   // copy — toStateVector is non-const
    astro::PosState s = oe.toStateVector(et);

    // Rotate J2000 equatorial → ECLIPJ2000 (Rx by Earth's obliquity).
    const double ce = std::cos(kObliquity), se = std::sin(kObliquity);
    auto rx = [&](const astro::Vec3& v) -> astro::Vec3 {
        return { v.x, ce*v.y + se*v.z, -se*v.y + ce*v.z };
    };
    return { rx(s.r), rx(s.v) };
}

} // namespace spacecraft
