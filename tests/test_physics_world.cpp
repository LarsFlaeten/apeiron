#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "apeiron/spacecraft/PhysicsWorld.h"
#include "apeiron/spacecraft/Spacecraft.h"

#include <cmath>
#include <numbers>

// Helper: construct a test spacecraft at given ECI position/velocity
static Spacecraft makeSpacecraft(const glm::dvec3& posKm, const glm::dvec3& velKmps)
{
    astro::State s;
    s.P.r = posKm;
    s.P.v = velKmps;
    const glm::dmat3 I = glm::dmat3(1000.0);  // diagonal 1000 kg·m²
    return Spacecraft(1000.0, I, s);
}

// ---------------------------------------------------------------------------
// FakeEphemeris — returns hand-crafted circular orbits; no SPICE kernels.
//
// Bodies:
//   SUN   (10): at origin (geocentric → large distance not needed for tests)
//   EARTH (399): always at (0,0,0) — convention matches PhysicsWorld
//   MOON  (301): circular orbit at 384400 km
// ---------------------------------------------------------------------------
struct FakeEphemeris final : public spacecraft::IEphemeris {
    astro::PosState getState(int naif, const astro::EphemerisTime& et) const override
    {
        astro::PosState s;
        if (naif == 301) {
            // Moon: circular orbit r=384400 km, T≈27.3 days
            const double T   = 27.3 * 86400.0;
            const double r   = 384400.0;
            const double w   = 2.0 * std::numbers::pi / T;
            const double ang = w * et.getETValue();
            s.r = { r * std::cos(ang), r * std::sin(ang), 0.0 };
            s.v = { -r * w * std::sin(ang), r * w * std::cos(ang), 0.0 };
        } else if (naif == 10) {
            // Sun: ~150e6 km from Earth (just needs to be far)
            s.r = { 149597870.7, 0.0, 0.0 };
            s.v = {};
        }
        // EARTH (399) / anything else → zero (Earth is at ECI origin)
        return s;
    }
};

// Gravity constants (km³/s²)
static constexpr double kGmSun   = 1.32712440018e11;
static constexpr double kGmEarth = 398600.4418;
static constexpr double kGmMoon  = 4902.800118;

// Radii (km)
static constexpr double kReEarth = 6371.0;
static constexpr double kReMoon  = 1737.4;

static std::vector<spacecraft::PhysicsWorld::BodySpec> makeSpecs()
{
    return {
        { "SUN",   10,  kGmSun,   695700.0 },
        { "EARTH", 399, kGmEarth, kReEarth },
        { "MOON",  301, kGmMoon,  kReMoon  },
    };
}

// ---------------------------------------------------------------------------
// 1. SOI radius — Hill-sphere formula
// ---------------------------------------------------------------------------
TEST_CASE("PhysicsWorld SOI radii match Hill-sphere formula", "[physics_world]")
{
    FakeEphemeris eph;
    spacecraft::PhysicsWorld pw(makeSpecs(), "EARTH", eph);

    // EARTH SOI: derived from Sun position (FakeEphemeris returns Sun at 149597870.7 km).
    // r_SOI_earth = a_earth_sun * (GM_earth / GM_sun)^(2/5)
    const double aEarthSun    = 149597870.7;
    const double earthSoiCalc = aEarthSun * std::pow(kGmEarth / kGmSun, 2.0 / 5.0);
    auto* earth = pw.findBody("EARTH");
    REQUIRE(earth != nullptr);
    CHECK_THAT(earth->soiKm, Catch::Matchers::WithinRel(earthSoiCalc, 1e-6));

    // Moon SOI: a_moon = 384400 km from Earth; Moon orbits Earth so parent is Earth.
    // r_SOI = a * (GM_moon / GM_earth)^(2/5)
    const double aMoon   = 384400.0;
    const double soiCalc = aMoon * std::pow(kGmMoon / kGmEarth, 2.0 / 5.0);
    auto* moon = pw.findBody("MOON");
    REQUIRE(moon != nullptr);
    CHECK_THAT(moon->soiKm, Catch::Matchers::WithinRel(soiCalc, 1e-6));
}

// ---------------------------------------------------------------------------
// 2. Default dominant body = EARTH after construction
// ---------------------------------------------------------------------------
TEST_CASE("PhysicsWorld dominant body defaults to startBodyName", "[physics_world]")
{
    FakeEphemeris eph;
    spacecraft::PhysicsWorld pw(makeSpecs(), "EARTH", eph);
    CHECK(pw.dominantBodyName() == "EARTH");
    CHECK(pw.refBody().name == "EARTH");
    CHECK_THAT(pw.refBody().mu, Catch::Matchers::WithinRel(kGmEarth, 1e-9));
}

// ---------------------------------------------------------------------------
// 3. SOI transition fires exactly once and latches
// ---------------------------------------------------------------------------
TEST_CASE("PhysicsWorld SOI transition fires once and latches", "[physics_world]")
{
    FakeEphemeris eph;
    spacecraft::PhysicsWorld pw(makeSpecs(), "EARTH", eph);

    // Moon is at (384400, 0, 0) in FakeEphemeris.
    // Moon SOI ≈ 66183 km (Earth-parent formula).
    // posECI is seeded from the ephemeris at construction so the very first
    // updateGravity call already uses the real Moon position.
    auto* moon = pw.findBody("MOON");
    REQUIRE(moon != nullptr);

    // Run one updateGravity with player at Earth centre — well outside Moon SOI.
    // Should NOT trigger a transition.
    const astro::EphemerisTime t0(0.0);
    auto evt0 = pw.updateGravity({}, glm::dvec3(0.0), t0);
    CHECK(evt0.changed == false);

    // Place player at Moon position (distance 0 from Moon → inside Moon SOI).
    glm::dvec3 moonPos(384400.0, 0.0, 0.0);

    auto evt1 = pw.updateGravity({}, moonPos, t0);
    CHECK(evt1.changed == true);
    CHECK(evt1.newRefBody.name == "MOON");
    CHECK(pw.dominantBodyName() == "MOON");

    // Second call at same position — should NOT fire again (already latched)
    auto evt2 = pw.updateGravity({}, moonPos, t0);
    CHECK(evt2.changed == false);
}

// ---------------------------------------------------------------------------
// 4. Attractor tidal flags — Earth tidal=false, Moon tidal=true (ECI mode)
// ---------------------------------------------------------------------------
TEST_CASE("PhysicsWorld ECI attractors: dominant body tidal=false, others tidal=true",
          "[physics_world]")
{
    FakeEphemeris eph;
    spacecraft::PhysicsWorld pw(makeSpecs(), "EARTH", eph);

    // Build a minimal spacecraft so we can inspect its attractors.
    auto sc = makeSpacecraft(glm::dvec3(7000.0, 0.0, 0.0),
                             glm::dvec3(0.0, 7.7, 0.0));  // LEO

    std::vector<Spacecraft*> scPtrs = { &sc };
    const astro::EphemerisTime t0(0.0);
    pw.updateGravity(scPtrs, sc.position(), t0);

    // After updateGravity in ECI mode the dominant body (EARTH) attractor
    // should have tidal=false; Moon and Sun should have tidal=true.
    const auto& atts = sc.attractors();
    REQUIRE(!atts.empty());

    bool foundEarth = false, foundMoon = false;
    for (const auto& a : atts) {
        if (std::abs(a.GM - kGmEarth) < 1.0) {
            CHECK(a.tidal == false);
            foundEarth = true;
        }
        if (std::abs(a.GM - kGmMoon) < 1.0) {
            CHECK(a.tidal == true);
            foundMoon = true;
        }
    }
    CHECK(foundEarth);
    CHECK(foundMoon);
}

// ---------------------------------------------------------------------------
// 5. Circular Earth orbit energy conservation over one orbit period
// ---------------------------------------------------------------------------
TEST_CASE("PhysicsWorld circular LEO energy conservation", "[physics_world]")
{
    FakeEphemeris eph;
    spacecraft::PhysicsWorld pw(makeSpecs(), "EARTH", eph);

    const double r0     = 6778.0;  // ~400 km altitude
    const double v_circ = std::sqrt(kGmEarth / r0);

    auto sc = makeSpacecraft(glm::dvec3(r0, 0.0, 0.0),
                             glm::dvec3(0.0, v_circ, 0.0));

    // Orbital period
    const double T = 2.0 * std::numbers::pi * std::sqrt(r0 * r0 * r0 / kGmEarth);

    // Specific orbital energy: ε = v²/2 - GM/r
    auto energy = [&](const Spacecraft& s) {
        double v2 = glm::dot(s.velocity(), s.velocity());
        double r  = glm::length(s.position());
        return 0.5 * v2 - kGmEarth / r;
    };

    const double e0 = energy(sc);

    std::vector<Spacecraft*> scPtrs = { &sc };
    const astro::EphemerisTime et(0.0);

    // Initialise attractors
    pw.updateGravity(scPtrs, sc.position(), et);

    // Integrate one full orbit in chunks of T/100
    const double chunk = T / 100.0;
    for (int i = 0; i < 100; ++i) {
        pw.integrate(chunk, et, scPtrs,
                     [](double) {},
                     [](double) {});
    }

    const double eF = energy(sc);

    // Energy should be conserved to within 0.1% (RK4 numerical drift over one orbit)
    CHECK_THAT(eF, Catch::Matchers::WithinRel(e0, 1e-3));
}

// ---------------------------------------------------------------------------
// 6. setDominantBody correctly updates refBody
// ---------------------------------------------------------------------------
TEST_CASE("PhysicsWorld setDominantBody updates refBody correctly", "[physics_world]")
{
    FakeEphemeris eph;
    spacecraft::PhysicsWorld pw(makeSpecs(), "EARTH", eph);

    REQUIRE(pw.dominantBodyName() == "EARTH");

    pw.setDominantBody("MOON");
    CHECK(pw.dominantBodyName() == "MOON");
    CHECK(pw.refBody().name == "MOON");
    CHECK_THAT(pw.refBody().mu, Catch::Matchers::WithinRel(kGmMoon, 1e-9));
    CHECK(pw.refBody().naifId == 301);

    // Unknown name — state must be unchanged
    pw.setDominantBody("BOGUS");
    CHECK(pw.dominantBodyName() == "MOON");
}
