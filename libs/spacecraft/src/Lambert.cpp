#include "apeiron/spacecraft/Lambert.h"

#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Izzo's Lambert solver.
//
// Parameterises by x ∈ (-1, +∞).  TOF uses the Lancaster–Blanchard formula
// (same as pykep/PyKEP lambert_problem.cpp):
//   a = 1/(1−x²)
//   α = 2·acos(x)                            (transfer parameter)
//   β = 2·asin(|λ|·√(1−x²)),  negated if λ<0
//   T̄ = a·√a·[(α−sinα)−(β−sinβ) + 2πN] / 2
//   T̄ = tof·√(2μ/s³)
//
// Velocity reconstruction from Izzo 2015 eqs. 17–20.
//
// Reference:
//   Izzo, D., "Revisiting Lambert's problem", Cel. Mech. Dyn. Astron. 121(1),
//   2015.  https://doi.org/10.1007/s10569-014-9587-y
// ---------------------------------------------------------------------------

namespace spacecraft {

namespace {

// Lancaster–Blanchard non-dimensional TOF.
static double tofND(double x, double lam, int N)
{
    const double a = 1.0 / (1.0 - x * x);

    if (a > 0.0) {
        // Elliptic
        const double alpha = 2.0 * std::acos(std::clamp(x, -1.0, 1.0));
        // sinβ/2 = |λ|·√(1−x²) = |λ|/√a  — always ≤ 1 since λ²≤1
        const double sinHB = std::clamp(std::abs(lam) * std::sqrt(1.0 - x * x), 0.0, 1.0);
        double beta = 2.0 * std::asin(sinHB);
        if (lam < 0.0) beta = -beta;
        return a * std::sqrt(a) * ((alpha - std::sin(alpha)) - (beta - std::sin(beta))
                                   + 2.0 * M_PI * N) * 0.5;
    } else {
        // Hyperbolic (x > 1) — only N=0 is physical
        const double alfa = 2.0 * std::acosh(x);
        const double arg  = std::sqrt(lam * lam * (x * x - 1.0));
        double beta = 2.0 * std::asinh(arg);
        if (lam < 0.0) beta = -beta;
        return -a * std::sqrt(-a) * ((beta - std::sinh(beta)) - (alfa - std::sinh(alfa))) * 0.5;
    }
}

// First derivative via centred 5-point stencil.
static double dtofND(double x, double lam, int N)
{
    const double h = 1e-5;
    return (-tofND(x + 2*h, lam, N)
            + 8.0 * tofND(x +   h, lam, N)
            - 8.0 * tofND(x -   h, lam, N)
            +       tofND(x - 2*h, lam, N)) / (12.0 * h);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
bool solveLambert(double            mu,
                  const glm::dvec3& r1,
                  const glm::dvec3& r2,
                  double            tof,
                  bool              prograde,
                  glm::dvec3&       v1,
                  glm::dvec3&       v2)
{
    // ---- Geometry ----
    const double r1m = glm::length(r1);
    const double r2m = glm::length(r2);
    if (r1m < 1e-9 || r2m < 1e-9 || tof <= 0.0) return false;

    const glm::dvec3 ir1 = r1 / r1m;
    const glm::dvec3 ir2 = r2 / r2m;

    double cosTA = std::clamp(glm::dot(ir1, ir2), -1.0, 1.0);
    bool ccw = (glm::cross(ir1, ir2).z >= 0.0);

    double ta = std::acos(cosTA);
    if (prograde) {
        if (!ccw) ta = 2.0 * M_PI - ta;
    } else {
        if (ccw)  ta = 2.0 * M_PI - ta;
    }

    if (ta < 1e-10 || std::abs(ta - 2.0 * M_PI) < 1e-10) return false;

    // ---- Izzo non-dimensional variables ----
    const double c    = std::sqrt(r1m*r1m + r2m*r2m - 2.0*r1m*r2m*cosTA);
    const double s    = (r1m + r2m + c) * 0.5;
    if (c < 1e-9) return false;

    const double lam2 = 1.0 - c / s;
    if (lam2 < 0.0) return false;
    double lam = std::sqrt(lam2);
    if (ta > M_PI) lam = -lam;   // long-way transfer

    const double tStar = tof * std::sqrt(2.0 * mu / (s * s * s));

    // ---- Newton–Raphson from x = 0 ----
    double x = 0.0;

    constexpr int    kMaxIter = 100;
    constexpr double kTolT   = 1e-11;

    for (int i = 0; i < kMaxIter; ++i) {
        double dt = tofND(x, lam, 0) - tStar;
        if (std::abs(dt) < kTolT) break;

        double dT = dtofND(x, lam, 0);
        if (std::abs(dT) < 1e-15) return false;

        double dx = -dt / dT;

        // Keep x in elliptic domain (guard both walls).
        const double xMin = -1.0 + 1e-7;
        const double xMax =  1.0 - 1e-7;
        x = std::clamp(x + dx, xMin, xMax);
    }

    if (std::abs(tofND(x, lam, 0) - tStar) > 1e-6) return false;

    // ---- Velocity reconstruction (Izzo 2015 eqs. 17–20) ----
    const double y     = std::sqrt(std::max(0.0, 1.0 - lam * lam * (1.0 - x * x)));
    const double gamma = std::sqrt(mu * s * 0.5);
    const double rho   = (r1m - r2m) / c;
    const double sigma = std::sqrt(std::max(0.0, 1.0 - rho * rho));

    const double vr1 =  gamma * ((lam * y - x) - rho * (lam * y + x)) / r1m;
    const double vr2 = -gamma * ((lam * y - x) + rho * (lam * y + x)) / r2m;
    const double vt1 =  gamma * sigma * (y + lam * x) / r1m;
    const double vt2 =  gamma * sigma * (y + lam * x) / r2m;

    // Orbit normal: for short-way (lam≥0) use cross(r1,r2); for long-way (lam<0)
    // the angular momentum is anti-parallel to that cross product, so flip it.
    const glm::dvec3 crossRR  = glm::cross(ir1, ir2);
    const double     crossLen = glm::length(crossRR);
    glm::dvec3 ih;
    if (crossLen < 1e-9) {
        // 180° degenerate — choose any perpendicular to r1.
        glm::dvec3 ref = (std::abs(ir1.z) < 0.9)
                       ? glm::dvec3(0, 0, 1)
                       : glm::dvec3(1, 0, 0);
        ih = glm::normalize(glm::cross(ir1, ref));
    } else {
        ih = crossRR / crossLen;
    }
    // For long-way transfers the transfer orbit angular momentum is anti-parallel
    // to cross(r1,r2), so flip ih to keep tangent vectors pointing prograde.
    if (lam < 0.0) ih = -ih;

    const glm::dvec3 it1 = glm::cross(ih, ir1);
    const glm::dvec3 it2 = glm::cross(ih, ir2);

    v1 = vr1 * ir1 + vt1 * it1;
    v2 = vr2 * ir2 + vt2 * it2;

    return true;
}

} // namespace spacecraft
