#include "apeiron/spacecraft/FiniteBurnPredictor.h"

#include <glm/glm.hpp>
#include <cmath>
#include <limits>

namespace spacecraft {

// ---------------------------------------------------------------------------
// Single RK4 step for gravity + optional retrograde thrust.
static void rk4Step(glm::dvec3& r, glm::dvec3& v,
                    double mu, double accelKmS2,
                    double dt, bool thrusting)
{
    auto deriv = [&](const glm::dvec3& ri, const glm::dvec3& vi)
        -> std::pair<glm::dvec3, glm::dvec3>
    {
        const double rm3 = std::pow(glm::dot(ri, ri), 1.5);
        glm::dvec3 a = -(mu / rm3) * ri;
        if (thrusting) {
            const double vl = glm::length(vi);
            if (vl > 1e-9)
                a -= (accelKmS2 / vl) * vi;
        }
        return {vi, a};
    };

    auto [dr1, da1] = deriv(r,                v               );
    auto [dr2, da2] = deriv(r + 0.5*dt*dr1,   v + 0.5*dt*da1  );
    auto [dr3, da3] = deriv(r + 0.5*dt*dr2,   v + 0.5*dt*da2  );
    auto [dr4, da4] = deriv(r +     dt*dr3,   v +     dt*da3  );

    r += (dt / 6.0) * (dr1 + 2.0*dr2 + 2.0*dr3 + dr4);
    v += (dt / 6.0) * (da1 + 2.0*da2 + 2.0*da3 + da4);
}

// ---------------------------------------------------------------------------
std::pair<glm::dvec3, glm::dvec3> propagateFiniteBurn(
    glm::dvec3 r0, glm::dvec3 v0,
    double mu, double accelKmS2,
    double tCoast, int nCoastSteps,
    double tBurn,  int nBurnSteps)
{
    glm::dvec3 r = r0, v = v0;

    if (nCoastSteps > 0 && tCoast > 0.0) {
        const double dt = tCoast / nCoastSteps;
        for (int i = 0; i < nCoastSteps; ++i)
            rk4Step(r, v, mu, accelKmS2, dt, false);
    }

    if (nBurnSteps > 0 && tBurn > 0.0) {
        const double dt = tBurn / nBurnSteps;
        for (int i = 0; i < nBurnSteps; ++i)
            rk4Step(r, v, mu, accelKmS2, dt, true);
    }

    return {r, v};
}

// ---------------------------------------------------------------------------
double periapsisRadius(const glm::dvec3& r, const glm::dvec3& v, double mu)
{
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    const double v2     = glm::dot(v, v);
    const double rMag   = glm::length(r);
    if (rMag < 1.0) return kNaN;

    const double energy = 0.5 * v2 - mu / rMag;
    if (energy >= 0.0) return kNaN;   // not captured

    const glm::dvec3 hv = glm::cross(r, v);
    const double h2 = glm::dot(hv, hv);
    const double e2 = std::max(0.0, 1.0 + 2.0 * energy * h2 / (mu * mu));
    return h2 / (mu * (1.0 + std::sqrt(e2)));
}

} // namespace spacecraft
