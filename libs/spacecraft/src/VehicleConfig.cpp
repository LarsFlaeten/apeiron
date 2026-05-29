#include "apeiron/spacecraft/VehicleConfig.h"

#include <astro/SpiceCore.h>
#include <toml++/toml.hpp>
#include <cmath>
#include <stdexcept>

namespace spacecraft {

static constexpr double kDeg = M_PI / 180.0;

// Earth obliquity — used only for Earth TLE-derived elements (equatorial→ecliptic).
static constexpr double kObliquityEarth = 23.4393 * kDeg;

// Resolve a SPICE body name to a NAIF integer id via the astro wrapper.
static int resolveNaifId(const std::string& bodyName)
{
    try { return astro::Spice().bodyNameToId(bodyName); }
    catch (const std::exception& ex) {
        throw std::runtime_error("VehicleConfig: unknown central_body \""
                                 + bodyName + "\": " + ex.what());
    }
}

// Query GM (km³/s²) for a NAIF body id via the astro wrapper.
static double queryGM(int naifId)
{
    double mu = 0.0;
    try { astro::Spice().getPlanetaryConstants(naifId, "GM", mu); }
    catch (const std::exception& ex) {
        throw std::runtime_error("VehicleConfig: could not retrieve GM for NAIF id "
                                 + std::to_string(naifId) + ": " + ex.what());
    }
    if (mu <= 0.0)
        throw std::runtime_error("VehicleConfig: GM is zero for NAIF id "
                                 + std::to_string(naifId));
    return mu;
}

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

    // [orbit] is no longer part of spacecraft files — orbit comes from the
    // scenario via applyOrbit(). Silently ignore any stale [orbit] section.

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

void applyOrbit(VehicleConfig&     vc,
                const std::string& centralBody,
                const std::string& epochStr,
                double             incDeg,
                double             raanDeg,
                double             eccentricity,
                double             argPerigeeDeg,
                double             meanAnomalyDeg,
                double             meanMotionRevDay)
{
    if (epochStr.empty())
        throw std::runtime_error("applyOrbit: epoch string is empty");

    vc.centralBody = centralBody;

    const int    naifId = resolveNaifId(centralBody);
    const double mu     = queryGM(naifId);

    const double inc   = incDeg        * kDeg;
    const double raan  = raanDeg       * kDeg;
    const double e     = eccentricity;
    const double argpe = argPerigeeDeg * kDeg;
    const double M0    = meanAnomalyDeg * kDeg;
    const double n_rads = meanMotionRevDay * 2.0 * M_PI / 86400.0;
    const double a      = std::cbrt(mu / (n_rads * n_rads));
    const double h      = std::sqrt(mu * a * (1.0 - e * e));

    astro::OrbitElements& oe = vc.orbitElements;
    oe.h     = h;
    oe.i     = inc;
    oe.omega = raan;
    oe.e     = e;
    oe.w     = argpe;
    oe.M0    = M0;
    oe.epoch = astro::EphemerisTime::fromString(epochStr);
    oe.mu    = mu;
    oe.computeDerivedQuantities();

    vc.hasOrbit = true;
}

astro::PosState vehicleStateAtEt(const VehicleConfig& vc,
                                  const astro::EphemerisTime& et)
{
    astro::OrbitElements oe = vc.orbitElements;   // copy — toStateVector is non-const
    astro::PosState s = oe.toStateVector(et);

    if (vc.centralBody == "EARTH") {
        // TLE elements are given in TEME (near-equatorial); rotate to ECLIPJ2000.
        const double ce = std::cos(kObliquityEarth), se = std::sin(kObliquityEarth);
        auto rx = [&](const astro::Vec3& v) -> astro::Vec3 {
            return { v.x, ce*v.y + se*v.z, -se*v.y + ce*v.z };
        };
        return { rx(s.r), rx(s.v) };
    }

    // Non-Earth body: elements are defined in ECLIPJ2000 (inclination relative to
    // the ecliptic plane).  Add the central body's geocentric ECLIPJ2000 state so
    // the result is Earth-centred, matching the simulation reference frame.
    static const astro::ReferenceFrame kEclip = astro::ReferenceFrame::createEclipJ2000();
    int naifId = resolveNaifId(vc.centralBody);
    astro::PosState bodyState;
    try {
        astro::Spice().getRelativeGeometricState(
            static_cast<int>(naifId), 399, et, bodyState, kEclip);
    } catch (const std::exception& ex) {
        throw std::runtime_error(
            std::string("vehicleStateAtEt: could not get state for central_body \"")
            + vc.centralBody + "\": " + ex.what());
    }

    return {
        { s.r.x + bodyState.r.x, s.r.y + bodyState.r.y, s.r.z + bodyState.r.z },
        { s.v.x + bodyState.v.x, s.v.y + bodyState.v.y, s.v.z + bodyState.v.z }
    };
}

} // namespace spacecraft
