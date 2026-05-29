#include "apeiron/spacecraft/Spacecraft.h"

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

void Spacecraft::clearAttractors()
{
    m_ode.clearAttractors();
}

const std::vector<astro::Attractor>& Spacecraft::attractors() const
{
    return m_ode.getAttractors();
}

void Spacecraft::setBodyForce(const glm::dvec3& forceN)
{
    m_baseForceN = forceN;
    // Actual ODE force will be applied (with current attitude) in update().
}

void Spacecraft::setBodyTorque(const glm::dvec3& torqueNm)
{
    m_baseTorqueNm = torqueNm;
    // Actual RotODE torque will be applied in update().
}

void Spacecraft::update(double dt_s, const astro::EphemerisTime& et)
{
    astro::TimeDelta dt(dt_s);

    // Combine base (per-frame) and extra (per-tick constraint) forces, then apply.
    // Re-rotating each tick ensures the inertial-frame force tracks attitude changes
    // correctly even when multiple physics ticks run per render frame.
    m_ode.setBodyForce((m_baseForceN + m_extraForceN) * kNtoKm, m_state.R.q);
    m_rotOde.setBodyTorque(m_baseTorqueNm + m_extraTorqueNm);

    // Extra forces are consumed each tick; base forces persist until next setBodyForce.
    m_extraForceN   = glm::dvec3{0.0};
    m_extraTorqueNm = glm::dvec3{0.0};

    // Integrate translational state.
    auto transResult = astro::RKF78::doStep(m_ode, m_state.P, et, dt);
    m_state.P = transResult.s;

    // Integrate rotational state.
    auto rotResult = astro::PCDM::doStep(m_rotOde, m_state.R, et, dt);
    m_state.R = rotResult.rs;

    // Carry the adaptive step hint forward.
    m_dtHint = transResult.dt_next;
}
