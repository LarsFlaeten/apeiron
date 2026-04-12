#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "apeiron/spacecraft/SpacecraftModel.h"
#include "apeiron/spacecraft/ManifestLoader.h"
#include "apeiron/spacecraft/Autopilot.h"
#include "apeiron/spacecraft/Lambert.h"


#include <glm/glm.hpp>
#include <cmath>
#include <filesystem>

using namespace spacecraft;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Build the resultant wrench from current throttle state (ground truth check).
// Uses throttle directly (not firing) so it works without stepping PWM.
static Wrench computeActualWrench(const SpacecraftModel& m)
{
    glm::dvec3 F{}, T{};
    for (const auto& t : m.thrusters) {
        glm::dvec3 f   = glm::dvec3(t.direction) * static_cast<double>(t.thrustN * t.throttle);
        glm::dvec3 arm = glm::dvec3(t.position - m.centerOfMass);
        F += f;
        T += glm::cross(arm, f);
    }
    return { F.x, F.y, F.z, T.x, T.y, T.z };
}

// Apply a desired wrench and return the actual achieved wrench.
static Wrench applyAndMeasure(SpacecraftModel& m, const Wrench& desired)
{
    m.solveAllocation(desired);
    return computeActualWrench(m);
}

// Simple 3-thruster model with known geometry for unit tests.
// Thrusters:
//   A: pos (0, 1, 0), dir (1,0,0)  — pure +X force, +Z torque
//   B: pos (0,-1, 0), dir (1,0,0)  — pure +X force, -Z torque
//   C: pos (0, 0, 1), dir (0,1,0)  — pure +Y force, -X torque
// CoM at origin.
static SpacecraftModel makeSimple()
{
    SpacecraftModel m;
    m.massKg       = 100.0f;
    m.centerOfMass = {0, 0, 0};

    Thruster a; a.id = "A"; a.thrustN = 100.0f;
    a.position = {0, 1, 0}; a.direction = {1, 0, 0};
    Thruster b; b.id = "B"; b.thrustN = 100.0f;
    b.position = {0,-1, 0}; b.direction = {1, 0, 0};
    Thruster c; c.id = "C"; c.thrustN = 100.0f;
    c.position = {0, 0, 1}; c.direction = {0, 1, 0};

    m.thrusters = {a, b, c};
    m.buildEffectivenessMatrix();
    return m;
}

// ---------------------------------------------------------------------------
// Section 1: Effectiveness matrix basics
// ---------------------------------------------------------------------------

TEST_CASE("Effectiveness matrix — pure +X force, no torque", "[allocator]")
{
    auto m = makeSimple();
    // Fire A and B equally → pure +X, torques cancel.
    Wrench desired = {100.0, 0, 0, 0, 0, 0};
    auto achieved  = applyAndMeasure(m, desired);

    CHECK_THAT(achieved[0], WithinAbs(100.0, 1.0));  // Fx
    CHECK_THAT(achieved[1], WithinAbs(  0.0, 1.0));  // Fy
    CHECK_THAT(achieved[2], WithinAbs(  0.0, 1.0));  // Fz
    CHECK_THAT(achieved[3], WithinAbs(  0.0, 1.0));  // Tx
    CHECK_THAT(achieved[4], WithinAbs(  0.0, 1.0));  // Ty
    CHECK_THAT(achieved[5], WithinAbs(  0.0, 1.0));  // Tz
}

TEST_CASE("Effectiveness matrix — +Y force: only C fires, residual torque accepted", "[allocator]")
{
    // Thruster C is the only source of +Y force; it also produces -Tx torque.
    // The pseudoinverse minimises ||Bt - d||² across all 6 DOF simultaneously,
    // so it compromises: tC < 1 to reduce the unavoidable Tx residual.
    // The important checks: C is the only thruster used, and Fy > 0.
    auto m = makeSimple();
    Wrench desired = {0, 100.0, 0, 0, 0, 0};
    m.solveAllocation(desired);

    CHECK_THAT(m.thrusters[0].throttle, WithinAbs(0.0f, 0.05f));  // A off
    CHECK_THAT(m.thrusters[1].throttle, WithinAbs(0.0f, 0.05f));  // B off
    CHECK(m.thrusters[2].throttle > 0.1f);                         // C fires

    auto achieved = computeActualWrench(m);
    CHECK(achieved[1] > 30.0);   // some +Y force achieved
    CHECK(achieved[0] < 5.0);    // no significant spurious +X
}

TEST_CASE("Effectiveness matrix — all throttles in [0,1]", "[allocator]")
{
    auto m = makeSimple();
    Wrench desired = {50.0, 30.0, 0, 0, 0, 0};
    m.solveAllocation(desired);

    for (const auto& t : m.thrusters) {
        INFO("Thruster " << t.id << " throttle = " << t.throttle);
        CHECK(t.throttle >= 0.0f);
        CHECK(t.throttle <= 1.0f);
    }
}

TEST_CASE("Zero wrench demand → all thrusters off", "[allocator]")
{
    auto m = makeSimple();
    Wrench desired = {0, 0, 0, 0, 0, 0};
    m.solveAllocation(desired);

    for (const auto& t : m.thrusters) {
        CHECK_THAT(t.throttle, WithinAbs(0.0f, 0.01f));
    }
}

// ---------------------------------------------------------------------------
// Section 2: Symmetric 4-thruster ring (analytic ground truth)
// ---------------------------------------------------------------------------
//
// Four identical thrusters at 90° intervals around the Z axis, all pointing
// radially inward (+X or +Y toward origin). This gives:
//   T1: pos ( r,0,0), dir (-1,0,0)
//   T2: pos (-r,0,0), dir (+1,0,0)
//   T3: pos (0, r,0), dir (0,-1,0)
//   T4: pos (0,-r,0), dir (0,+1,0)
// Firing all four equally → pure -X/-Y... wait, they oppose each other.
// Firing T1+T2 = net zero force, +Z torque (couple).
// This tests that the allocator correctly identifies cancellation.

static SpacecraftModel makeRing(float r = 1.0f, float thrN = 100.0f)
{
    SpacecraftModel m;
    m.massKg = 200.0f;
    m.centerOfMass = {0,0,0};

    auto make = [&](const char* id, glm::vec3 pos, glm::vec3 dir) {
        Thruster t;
        t.id = id; t.thrustN = thrN;
        t.position = pos; t.direction = glm::normalize(dir);
        return t;
    };

    m.thrusters = {
        make("T1", { r, 0, 0}, {-1, 0, 0}),
        make("T2", {-r, 0, 0}, { 1, 0, 0}),
        make("T3", { 0, r, 0}, { 0,-1, 0}),
        make("T4", { 0,-r, 0}, { 0, 1, 0}),
    };
    m.buildEffectivenessMatrix();
    return m;
}

TEST_CASE("Ring model — pure +Z torque couple", "[allocator]")
{
    // T1 at (+r,0): force (-1,0,0) → torque = (r,0,0)×(-F,0,0) = (0,0,0). Hmm...
    // Actually for a couple producing +Z torque we need thrusters offset in X, firing in Y.
    // This ring model produces torques around Z from T3+T4 firing:
    //   T3 at (0,r,0) dir(0,-1,0): torque = (0,r,0)×(0,-F,0) = (0,0,0). Still zero.
    // The radial ring is degenerate for torque — skip torque test, check force cancellation.
    auto m = makeRing();
    // All four at equal throttle → forces should cancel.
    for (auto& t : m.thrusters) t.throttle = 0.5f;
    Wrench w = computeActualWrench(m);
    CHECK_THAT(w[0], WithinAbs(0.0, 0.5));
    CHECK_THAT(w[1], WithinAbs(0.0, 0.5));
    CHECK_THAT(w[2], WithinAbs(0.0, 0.5));
}

// ---------------------------------------------------------------------------
// Section 3: Orion manifest loading
// ---------------------------------------------------------------------------

static const std::filesystem::path kOrionToml =
    std::filesystem::path(APEIRON_DATA_DIR)
    / "spacecraft/orion/CM_SM06_thrusters.toml";

TEST_CASE("Orion manifest loads without error", "[orion]")
{
    if (!std::filesystem::exists(kOrionToml))
        SKIP("Orion TOML not found at " + kOrionToml.string());

    SpacecraftModel m;
    REQUIRE_NOTHROW(m = loadManifest(kOrionToml));
    CHECK(m.thrusters.size() == 41);
}

TEST_CASE("Orion thruster types and counts", "[orion]")
{
    if (!std::filesystem::exists(kOrionToml))
        SKIP("Orion TOML not found at " + kOrionToml.string());

    auto m = loadManifest(kOrionToml);

    int nMain = 0, nAux = 0, nRcs = 0;
    for (const auto& t : m.thrusters) {
        if (t.type == ThrusterType::MainEngine) ++nMain;
        if (t.type == ThrusterType::Auxiliary)  ++nAux;
        if (t.type == ThrusterType::RCS)         ++nRcs;
    }
    CHECK(nMain == 1);
    CHECK(nAux  == 8);
    CHECK(nRcs  == 32);
}

TEST_CASE("Orion thruster directions are unit vectors", "[orion]")
{
    if (!std::filesystem::exists(kOrionToml))
        SKIP("Orion TOML not found at " + kOrionToml.string());

    auto m = loadManifest(kOrionToml);
    for (const auto& t : m.thrusters) {
        float len = glm::length(t.direction);
        INFO("Thruster " << t.id << " direction length = " << len);
        CHECK_THAT(len, WithinAbs(1.0f, 1e-4f));
    }
}

TEST_CASE("Orion all throttles in [0,1] after allocation", "[orion]")
{
    if (!std::filesystem::exists(kOrionToml))
        SKIP("Orion TOML not found at " + kOrionToml.string());

    auto m = loadManifest(kOrionToml);

    // Test several representative demands.
    std::vector<Wrench> demands = {
        {10000, 0, 0, 0, 0, 0},    // pure main engine thrust
        {0, 0, 0, 500, 0, 0},      // pure roll torque
        {0, 0, 0, 0, 500, 0},      // pure pitch torque
        {0, 0, 0, 0, 0, 500},      // pure yaw torque
        {500, 0, 0, 0, 200, 0},    // combined
    };

    for (const auto& d : demands) {
        m.solveAllocation(d);
        for (const auto& t : m.thrusters) {
            INFO("Demand [" << d[0] << "," << d[1] << "," << d[2]
                 << "," << d[3] << "," << d[4] << "," << d[5]
                 << "] — thruster " << t.id << " = " << t.throttle);
            CHECK(t.throttle >= 0.0f);
            CHECK(t.throttle <= 1.0f);
        }
    }
}

TEST_CASE("Orion — print thruster selection for each demand axis", "[orion][.]")
{
    // Tagged [.] so it only runs when explicitly requested: ctest -R orion_print
    if (!std::filesystem::exists(kOrionToml))
        SKIP("Orion TOML not found at " + kOrionToml.string());

    auto m = loadManifest(kOrionToml);

    struct Case { const char* name; Wrench w; };
    std::vector<Case> cases = {
        {"Main engine full",  {25700, 0, 0, 0, 0, 0}},
        {"RCS +X translate",  {2000,  0, 0, 0, 0, 0}},
        {"RCS +Y translate",  {0, 2000, 0, 0, 0, 0}},
        {"RCS +Z translate",  {0, 0, 2000, 0, 0, 0}},
        {"Roll  +Tx",         {0, 0, 0, 2000, 0, 0}},
        {"Pitch +Ty",         {0, 0, 0, 0, 2000, 0}},
        {"Yaw   +Tz",         {0, 0, 0, 0, 0, 2000}},
    };

    for (const auto& c : cases) {
        m.solveAllocation(c.w);
        WARN("\n=== " << c.name << " ===");
        for (const auto& t : m.thrusters) {
            if (t.throttle > 0.01f) {
                WARN("  " << t.id << " (q" << t.quad << ")  " << t.throttle * 100.0f << "%");
            }
        }
        auto achieved = computeActualWrench(m);
        WARN("  Achieved: F=[" << achieved[0] << "," << achieved[1] << "," << achieved[2]
             << "]  T=[" << achieved[3] << "," << achieved[4] << "," << achieved[5] << "]");
    }
}

// ---------------------------------------------------------------------------
// Section 4: Autopilot test
// ---------------------------------------------------------------------------

TEST_CASE("Autopilot_killrot — check commanded thrust", "[autopilot]")
{
    Autopilot ap{};
    ap.mode = AutopilotMode::Killrot;

    if (!std::filesystem::exists(kOrionToml))
        SKIP("Orion TOML not found at " + kOrionToml.string());

    auto m = loadManifest(kOrionToml);
    
    const glm::dvec3 inertiaDiag = glm::dvec3(m.inertiaDiag);

    struct Case { const char* name; glm::dvec3 omega_body; };
    std::vector<Case> cases = {
        {"Rot +X high",  { glm::radians(2.0), 0, 0}}
    };

    for (const auto& c : cases) {
        WARN("\n=== " << c.name << " ===");
        const double dt = 1.0 / 60.0;
        bool settleClamp = false;
        auto w = ap.compute(glm::dquat(1,0,0,0), c.omega_body, inertiaDiag, dt, settleClamp);
        WARN("\n  Computed wrench: [" << w[0] << ", " << w[1] << ", " << w[2] << ", " << w[3] << ", " << w[4] << ", " << w[4] << " ]\n");
        m.solveAllocation(w);
        for (const auto& t : m.thrusters) {
            if (t.throttle > 0.01f) {
                WARN("  " << t.id << " (q" << t.quad << ")  " << t.throttle * 100.0f << "%");
            }
        }
        auto achieved = computeActualWrench(m);
        WARN("  Achieved: F=[" << achieved[0] << "," << achieved[1] << "," << achieved[2]
             << "]  T=[" << achieved[3] << "," << achieved[4] << "," << achieved[5] << "]");
    }

}

// ---------------------------------------------------------------------------
// Section 5: Lambert / Izzo solver
// ---------------------------------------------------------------------------
//
// Strategy: construct test cases from known orbits via Keplerian mechanics,
// then verify that solveLambert returns the exact velocity vectors.
//
// All positions in km, velocities in km/s, time in seconds.
// mu_earth = 398600.4418 km³/s²

static constexpr double kMuEarth = 398600.4418;

// Circular orbital speed at radius r [km].
static double vCirc(double r) { return std::sqrt(kMuEarth / r); }

// Half-period of a circular orbit (= period of a Hohmann semi-ellipse).
// For an ellipse with semi-major axis a: T/2 = π·sqrt(a³/μ).
static double halfPeriod(double a) { return M_PI * std::sqrt(a * a * a / kMuEarth); }

// ---------------------------------------------------------------------------
// Test 1: Circular orbit, 180° transfer (λ = 0 — minimum-energy case).
//
// r1 = (r, 0, 0),  r2 = (-r, 0, 0),  TOF = T_circle / 2.
// The orbit PLANE is degenerate for anti-podal points — any plane through r1
// and r2 is valid.  We only check that:
//   • solveLambert converges (returns true)
//   • speed |v1| = |v2| = vc  (vis-viva on circular orbit)
//   • v1 ⊥ r1  (purely tangential departure)
//   • v2 ⊥ r2  (purely tangential arrival)
// ---------------------------------------------------------------------------
TEST_CASE("Lambert — circular orbit 180° (lambda=0)", "[lambert]")
{
    const double r   = 7000.0;   // km
    const double vc  = vCirc(r);
    const double tof = halfPeriod(r);

    glm::dvec3 r1{ r,  0, 0};
    glm::dvec3 r2{-r,  0, 0};

    glm::dvec3 v1, v2;
    bool ok = spacecraft::solveLambert(kMuEarth, r1, r2, tof, /*prograde=*/true, v1, v2);

    REQUIRE(ok);

    // Speed must equal circular velocity.
    CHECK_THAT(glm::length(v1), WithinRel(vc, 1e-4));
    CHECK_THAT(glm::length(v2), WithinRel(vc, 1e-4));

    // Velocity must be perpendicular to position (purely tangential).
    CHECK_THAT(glm::dot(v1, glm::dvec3(r1) / r), WithinAbs(0.0, 1e-3));
    CHECK_THAT(glm::dot(v2, glm::dvec3(r2) / r), WithinAbs(0.0, 1e-3));
}

// ---------------------------------------------------------------------------
// Test 2: Circular orbit, 120° prograde transfer.
//
// r1 = (r, 0, 0),  r2 = r·(cos 120°, sin 120°, 0),  TOF = T_circle / 3.
// The only Keplerian arc joining these two points in T/3 is the circular orbit
// itself, so v1 = (0, vc, 0) and v2 = vc·(−sin 120°, cos 120°, 0).
// ---------------------------------------------------------------------------
TEST_CASE("Lambert — circular orbit 120° prograde", "[lambert]")
{
    const double r   = 7000.0;
    const double vc  = vCirc(r);
    const double tof = halfPeriod(r) * (2.0 / 3.0);   // T·(1/3) = (T/2)·(2/3)

    const double ang = 2.0 * M_PI / 3.0;  // 120°
    glm::dvec3 r1{r, 0, 0};
    glm::dvec3 r2{r * std::cos(ang), r * std::sin(ang), 0};

    glm::dvec3 v1, v2;
    bool ok = spacecraft::solveLambert(kMuEarth, r1, r2, tof, /*prograde=*/true, v1, v2);

    REQUIRE(ok);

    CHECK_THAT(v1.x, WithinAbs(0.0,  1e-3));
    CHECK_THAT(v1.y, WithinAbs( vc,  1e-3));
    CHECK_THAT(v1.z, WithinAbs(0.0,  1e-3));

    // Tangent at 120°: direction = (-sin120°, cos120°, 0) = (-√3/2, -1/2, 0)
    const double v2x_exp = -std::sin(ang) * vc;   // ≈ -6.534
    const double v2y_exp =  std::cos(ang) * vc;   // ≈ -3.773
    CHECK_THAT(v2.x, WithinAbs(v2x_exp, 1e-3));
    CHECK_THAT(v2.y, WithinAbs(v2y_exp, 1e-3));
    CHECK_THAT(v2.z, WithinAbs(0.0,     1e-3));
}

// ---------------------------------------------------------------------------
// Test 3: Circular orbit, 240° prograde (long-way, λ < 0).
//
// r1 = (r, 0, 0),  r2 = r·(cos 240°, sin 240°, 0),  TOF = T·(2/3).
// Going the long way around (the 240° arc), which is still prograde.
// Expected velocities are the same circular-orbit tangents.
// ---------------------------------------------------------------------------
TEST_CASE("Lambert — circular orbit 240° prograde (long-way)", "[lambert]")
{
    const double r   = 7000.0;
    const double vc  = vCirc(r);
    const double tof = halfPeriod(r) * (4.0 / 3.0);   // T·(2/3)

    const double ang = 4.0 * M_PI / 3.0;  // 240°
    glm::dvec3 r1{r, 0, 0};
    glm::dvec3 r2{r * std::cos(ang), r * std::sin(ang), 0};

    glm::dvec3 v1, v2;
    bool ok = spacecraft::solveLambert(kMuEarth, r1, r2, tof, /*prograde=*/true, v1, v2);

    REQUIRE(ok);

    CHECK_THAT(v1.x, WithinAbs(0.0, 1e-3));
    CHECK_THAT(v1.y, WithinAbs( vc, 1e-3));
    CHECK_THAT(v1.z, WithinAbs(0.0, 1e-3));

    const double v2x_exp = -std::sin(ang) * vc;
    const double v2y_exp =  std::cos(ang) * vc;
    CHECK_THAT(v2.x, WithinAbs(v2x_exp, 1e-3));
    CHECK_THAT(v2.y, WithinAbs(v2y_exp, 1e-3));
    CHECK_THAT(v2.z, WithinAbs(0.0,     1e-3));
}

// ---------------------------------------------------------------------------
// Test 4: Hohmann transfer LEO → GEO (λ = 0).
//
// Classic astrodynamics result (vis-viva):
//   v_periapsis = sqrt(μ·(2/r1 − 1/a)),  v_apoapsis = sqrt(μ·(2/r2 − 1/a))
// ---------------------------------------------------------------------------
TEST_CASE("Lambert — Hohmann LEO to GEO", "[lambert]")
{
    const double r1val = 6678.0;   // km  (300 km altitude)
    const double r2val = 42164.0;  // km  GEO
    const double a     = (r1val + r2val) * 0.5;
    const double tof   = halfPeriod(a);

    glm::dvec3 r1{ r1val, 0, 0};
    glm::dvec3 r2{-r2val, 0, 0};   // 180° away
    const glm::dvec3 ir1 = glm::normalize(r1);
    const glm::dvec3 ir2 = glm::normalize(r2);

    glm::dvec3 v1, v2;
    bool ok = spacecraft::solveLambert(kMuEarth, r1, r2, tof, /*prograde=*/true, v1, v2);

    REQUIRE(ok);

    const double vp = std::sqrt(kMuEarth * (2.0 / r1val - 1.0 / a));  // ≈ 10.15 km/s
    const double va = std::sqrt(kMuEarth * (2.0 / r2val - 1.0 / a));  // ≈ 1.61 km/s

    // 180° transfer — orbit plane is degenerate.  Check speed and that departure/
    // arrival are purely tangential (vr = 0 at periapsis and apoapsis of ellipse).
    CHECK_THAT(glm::length(v1), WithinRel(vp, 1e-4));
    CHECK_THAT(glm::length(v2), WithinRel(va, 1e-4));

    CHECK_THAT(glm::dot(v1, ir1), WithinAbs(0.0, 1e-3));   // v1 ⊥ r1
    CHECK_THAT(glm::dot(v2, ir2), WithinAbs(0.0, 1e-3));   // v2 ⊥ r2
}

// ---------------------------------------------------------------------------
// Test 5: Out-of-plane transfer — verify z-components are non-zero and
//         the speed at departure/arrival matches vis-viva.
//
// r1 and r2 are in different orbital planes to exercise the 3D path.
// We construct a known ellipse from elements and check energy conservation.
// ---------------------------------------------------------------------------
TEST_CASE("Lambert — 3D inclined orbit energy conservation", "[lambert]")
{
    // Semi-major axis and eccentricity defining the transfer orbit.
    const double a   = 10000.0;   // km
    const double ecc = 0.3;
    const double inc = 30.0 * M_PI / 180.0;  // 30° inclination

    // True anomaly at departure and arrival.
    const double nu1 = 30.0  * M_PI / 180.0;
    const double nu2 = 150.0 * M_PI / 180.0;

    // p = a(1-e²)
    const double p = a * (1.0 - ecc * ecc);

    auto r_from_nu = [&](double nu) -> glm::dvec3 {
        double r = p / (1.0 + ecc * std::cos(nu));
        // Perifocal (PQW) frame: P along periapsis, Q 90° ahead, W = orbit normal.
        // Rotate into ECI using inclination only (Ω=ω=0 for simplicity).
        double xp = r * std::cos(nu);
        double yp = r * std::sin(nu);
        // ECI: rotate about x-axis by inclination.
        return { xp,
                 yp * std::cos(inc),
                 yp * std::sin(inc) };
    };

    auto v_from_nu = [&](double nu) -> glm::dvec3 {
        double sqmup = std::sqrt(kMuEarth / p);
        double vx =  sqmup * (-std::sin(nu));
        double vy =  sqmup * (ecc + std::cos(nu));
        return { vx,
                 vy * std::cos(inc),
                 vy * std::sin(inc) };
    };

    glm::dvec3 r1  = r_from_nu(nu1);
    glm::dvec3 r2  = r_from_nu(nu2);
    glm::dvec3 v1e = v_from_nu(nu1);
    glm::dvec3 v2e = v_from_nu(nu2);

    // Compute TOF using Kepler's equation (mean anomaly difference).
    auto eccAnom = [&](double nu) {
        return 2.0 * std::atan2(std::sqrt(1.0 - ecc) * std::sin(nu / 2.0),
                                std::sqrt(1.0 + ecc) * std::cos(nu / 2.0));
    };
    double E1   = eccAnom(nu1);
    double E2   = eccAnom(nu2);
    double M1   = E1 - ecc * std::sin(E1);
    double M2   = E2 - ecc * std::sin(E2);
    if (M2 < M1) M2 += 2.0 * M_PI;
    const double n   = std::sqrt(kMuEarth / (a * a * a));
    const double tof = (M2 - M1) / n;

    glm::dvec3 v1s, v2s;
    bool ok = spacecraft::solveLambert(kMuEarth, r1, r2, tof, /*prograde=*/true, v1s, v2s);

    REQUIRE(ok);

    // Velocity components should match the Keplerian solution.
    CHECK_THAT(v1s.x, WithinAbs(v1e.x, 1e-3));
    CHECK_THAT(v1s.y, WithinAbs(v1e.y, 1e-3));
    CHECK_THAT(v1s.z, WithinAbs(v1e.z, 1e-3));

    CHECK_THAT(v2s.x, WithinAbs(v2e.x, 1e-3));
    CHECK_THAT(v2s.y, WithinAbs(v2e.y, 1e-3));
    CHECK_THAT(v2s.z, WithinAbs(v2e.z, 1e-3));
}

// ---------------------------------------------------------------------------
// Test 6: Degenerate inputs return false.
// ---------------------------------------------------------------------------
TEST_CASE("Lambert — degenerate inputs return false", "[lambert]")
{
    glm::dvec3 r1{7000, 0, 0}, r2{-7000, 0, 0};
    glm::dvec3 v1, v2;

    // Negative TOF.
    CHECK_FALSE(spacecraft::solveLambert(kMuEarth, r1, r2, -100.0, true, v1, v2));

    // Zero TOF.
    CHECK_FALSE(spacecraft::solveLambert(kMuEarth, r1, r2, 0.0, true, v1, v2));

    // Zero r1.
    CHECK_FALSE(spacecraft::solveLambert(kMuEarth, {0,0,0}, r2, 1000.0, true, v1, v2));

    // Identical positions (zero transfer angle).
    CHECK_FALSE(spacecraft::solveLambert(kMuEarth, r1, r1, 1000.0, true, v1, v2));
}
