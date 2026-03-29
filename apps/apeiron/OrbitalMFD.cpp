#include "OrbitalMFD.h"

#include <glm/glm.hpp>
#include <numbers>
#include <cmath>
#include <cstdio>

static constexpr double kRad2Deg = 180.0 / std::numbers::pi;

void OrbitalMFD::update(const astro::PosState&     state,
                         const astro::EphemerisTime& et,
                         double                      mu,
                         double                      bodyRadiusKm)
{
    m_altKm  = glm::length(state.r) - bodyRadiusKm;
    m_velKms = glm::length(state.v);

    auto oe = astro::OrbitElements::fromStateVector(state, et, mu);

    m_apoAltKm  = oe.ap - bodyRadiusKm;
    m_perAltKm  = oe.rp - bodyRadiusKm;
    m_incDeg    = oe.i     * kRad2Deg;
    m_raanDeg   = oe.omega * kRad2Deg;
    m_eccen     = oe.e;
    m_argpeDeg  = oe.w * kRad2Deg;

    // True anomaly — skip for near-circular orbits where M0 may be NaN
    // (Kepler's equation is ill-conditioned at e ≈ 0).
    constexpr double kMinEcc = 1.0e-6;
    if (oe.e >= kMinEcc && std::isfinite(oe.M0)) {
        double trueAnom = astro::OrbitElements::trueAnomalyFromMeanAnomaly(oe.M0, oe.e);
        m_trueDeg    = std::isfinite(trueAnom) ? trueAnom * kRad2Deg : 0.0;
        m_trueValid  = true;
    } else {
        m_trueValid = false;
    }

    m_periodMin = (oe.T > 0.0) ? oe.T / 60.0 : 0.0;
}

void OrbitalMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32  kGreen = IM_COL32(0, 220, 80, 255);
    const float  lineH  = ImGui::GetTextLineHeight() + 3.0f;
    const float  pad    = 5.0f;
    const float  valX   = origin.x + size.x - pad;  // right-align values here

    float y = origin.y + pad;

    auto row = [&](const char* label, const char* fmt, double val) {
        dl->AddText({ origin.x + pad, y }, kGreen, label);
        char buf[32];
        std::snprintf(buf, sizeof(buf), fmt, val);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ valX - tsz.x, y }, kGreen, buf);
        y += lineH;
    };

    row("Alt",   "%8.1f km",   m_altKm);
    row("Vel",   "%6.3f km/s", m_velKms);
    row("Apo",   "%8.1f km",   m_apoAltKm);
    row("Per",   "%8.1f km",   m_perAltKm);
    row("Inc",   "%6.2f\xc2\xb0",  m_incDeg);    // UTF-8 degree sign
    row("RAAN",  "%6.2f\xc2\xb0",  m_raanDeg);
    row("Ecc",   "%8.5f",      m_eccen);
    row("ArgPe", "%6.2f\xc2\xb0",  m_argpeDeg);
    if (m_trueValid)
        row("TA", "%6.2f\xc2\xb0", m_trueDeg);
    if (m_periodMin > 0.0)
        row("T", "%6.1f min",  m_periodMin);
}
