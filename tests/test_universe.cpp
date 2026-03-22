#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <glm/glm.hpp>

#include "apeiron/universe/KernelPool.h"
#include "apeiron/universe/Ephemeris.h"
#include "astro/Exceptions.h"
#include "astro/Time.h"

// APEIRON_KERNEL_DIR is injected by CMake as an absolute path to data/kernels/
#ifndef APEIRON_KERNEL_DIR
#  error "APEIRON_KERNEL_DIR not defined — set via target_compile_definitions in CMake"
#endif

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr double AU_KM = 149597870.700; // km per AU

// Load the standard kernel set once for the whole test session.
// SPICE's pool is a global singleton; duplicate loads are silently ignored.
static void ensureKernels()
{
    static bool loaded = false;
    if (loaded) return;

    auto& pool = apeiron::universe::KernelPool::instance();
    pool.load(APEIRON_KERNEL_DIR "/lsk/naif0012.tls");
    pool.load(APEIRON_KERNEL_DIR "/spk/de440s.bsp");
    pool.load(APEIRON_KERNEL_DIR "/pck/pck00011.tpc");
    pool.load(APEIRON_KERNEL_DIR "/pck/gm_de440.tpc");
    loaded = true;
}

// J2000 epoch: 2000-Jan-01 12:00:00 UTC
static astro::EphemerisTime j2000()
{
    return astro::EphemerisTime::fromString("2000-01-01T12:00:00");
}

// ---------------------------------------------------------------------------
// KernelPool
// ---------------------------------------------------------------------------

TEST_CASE("KernelPool: loaded kernels are tracked", "[universe][kernel]")
{
    ensureKernels();
    REQUIRE(apeiron::universe::KernelPool::instance().loaded().size() >= 4);
}

TEST_CASE("KernelPool: missing file throws SpiceException", "[universe][kernel]")
{
    REQUIRE_THROWS_AS(
        apeiron::universe::KernelPool::instance().load("no_such_kernel.tls"),
        astro::SpiceException
    );
}

// ---------------------------------------------------------------------------
// Ephemeris — positions at J2000
// ---------------------------------------------------------------------------

TEST_CASE("Mars barycenter distance matches JPL Horizons at J2000", "[universe][ephemeris]")
{
    ensureKernels();

    // JPL Horizons (de441), observer = SSB, target = Mars (499):
    //   delta = 1.38409427268327 AU
    // de440s vs de441 difference measured at ~590 km = 3.9e-6 AU
    // Tolerance set to 1e-4 AU (~15 000 km) to be robust across ephemeris versions.
    constexpr double expected_au = 1.38409427268327;
    constexpr double tol_au      = 1e-4;

    auto state    = apeiron::universe::getState("MARS BARYCENTER", j2000());
    double range  = glm::length(state.position) / AU_KM;

    REQUIRE_THAT(range, Catch::Matchers::WithinAbs(expected_au, tol_au));
}

TEST_CASE("Earth distance from SSB is ~1 AU at J2000", "[universe][ephemeris]")
{
    ensureKernels();

    // Earth is near perihelion in early January (~0.983 AU).
    auto state   = apeiron::universe::getState("EARTH", j2000());
    double range = glm::length(state.position) / AU_KM;

    REQUIRE_THAT(range, Catch::Matchers::WithinAbs(1.0, 0.02));
}

TEST_CASE("Earth orbital speed is ~30 km/s at J2000", "[universe][ephemeris]")
{
    ensureKernels();

    auto state  = apeiron::universe::getState("EARTH", j2000());
    double speed = glm::length(state.velocity);

    REQUIRE_THAT(speed, Catch::Matchers::WithinAbs(30.0, 1.0));
}

TEST_CASE("Sun is close to SSB (within 0.01 AU) at J2000", "[universe][ephemeris]")
{
    ensureKernels();

    // The Sun orbits the SSB due to planetary gravity; offset is typically
    // within ~1-2 solar radii (~1e6 km ≈ 0.007 AU).
    auto state   = apeiron::universe::getState("SUN", j2000());
    double range = glm::length(state.position) / AU_KM;

    REQUIRE_THAT(range, Catch::Matchers::WithinAbs(0.0, 0.01));
}

TEST_CASE("State observer parameter works: Earth relative to Sun", "[universe][ephemeris]")
{
    ensureKernels();

    auto state   = apeiron::universe::getState("EARTH", j2000(), "SUN");
    double range = glm::length(state.position) / AU_KM;

    // Earth-Sun distance in early January: ~0.983 AU
    REQUIRE_THAT(range, Catch::Matchers::WithinAbs(1.0, 0.02));
}

TEST_CASE("Light time is positive and physically plausible", "[universe][ephemeris]")
{
    ensureKernels();

    auto state = apeiron::universe::getState("MARS BARYCENTER", j2000());

    // Mars at ~1.38 AU: light travel time = 1.38 AU / c
    // c ≈ 299792 km/s → 1 AU / c ≈ 499 s → 1.38 AU → ~688 s
    REQUIRE(state.lightTime > 0.0);
    REQUIRE_THAT(state.lightTime, Catch::Matchers::WithinAbs(690.0, 30.0));
}

TEST_CASE("Unknown body name throws SpiceException", "[universe][ephemeris]")
{
    ensureKernels();

    REQUIRE_THROWS_AS(
        apeiron::universe::getState("NOTABODY", j2000()),
        astro::SpiceException
    );
}
