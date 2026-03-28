#include "Spacecraft.h"

#include <glm/gtc/quaternion.hpp>

// km conversion: force is supplied in N (= kg·m/s²); the ODE works in km.
// 1 m/s² = 1e-3 km/s²  →  force_kN = force_N * 1e-3
static constexpr double kNtoKm = 1.0e-3;

Spacecraft::Spacecraft(double              massKg,
                       const glm::dmat3&   inertiaKgM2,
                       const astro::State& initialState)
    : m_massKg(massKg)
    , m_state(initialState)
    , m_rotOde(inertiaKgM2)
    , m_dtHint(0.1)   // initial step hint: 0.1 s
{
    m_ode.setMass(massKg);
}

void Spacecraft::addAttractor(const astro::Attractor& a)
{
    m_ode.addAttractor(a);
}

void Spacecraft::setBodyForce(const glm::dvec3& forceN)
{
    // Convert N → km-consistent units, rotate body→inertial via attitude.
    m_ode.setBodyForce(forceN * kNtoKm, m_state.R.q);
}

void Spacecraft::setBodyTorque(const glm::dvec3& torqueNm)
{
    m_rotOde.setBodyTorque(torqueNm);
}

void Spacecraft::update(double dt_s, const astro::EphemerisTime& et)
{
    astro::TimeDelta dt(dt_s);

    // Integrate translational state.
    auto transResult = astro::RKF78::doStep(m_ode, m_state.P, et, dt);
    m_state.P = transResult.s;

    // Integrate rotational state.
    auto rotResult = astro::PCDM::doStep(m_rotOde, m_state.R, et, dt);
    m_state.R = rotResult.rs;

    // Carry the adaptive step hint forward (not used yet with fixed-step
    // physics, but useful when switching to adaptive integration later).
    m_dtHint = transResult.dt_next;
}
