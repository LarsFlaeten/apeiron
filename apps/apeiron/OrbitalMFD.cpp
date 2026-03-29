#include "OrbitalMFD.h"

#include <glm/glm.hpp>
#include <numbers>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <limits>

static constexpr double kPi      = std::numbers::pi;
static constexpr double kTwoPi   = 2.0 * kPi;
static constexpr double kRad2Deg = 180.0 / kPi;
static constexpr double kDeg2Rad = kPi / 180.0;
static constexpr double kMinEcc  = 1.0e-6;

// ---- palette -----------------------------------------------------------------
static const ImU32 kColGreen    = IM_COL32(  0, 210,  75, 210);
static const ImU32 kColDim      = IM_COL32(  0, 160,  55, 150);
static const ImU32 kColDivider  = IM_COL32(  0, 120,  40, 120);
static const ImU32 kColOrbit    = IM_COL32(  0, 200,  70, 200);
static const ImU32 kColBody     = IM_COL32( 30, 100, 200, 220);
static const ImU32 kColBodyRim  = IM_COL32( 60, 140, 255, 180);
static const ImU32 kColShip     = IM_COL32(220, 200,   0, 255);
static const ImU32 kColMark     = IM_COL32(  0, 180,  60, 160);
static const ImU32 kColApseLine = IM_COL32(  0, 140,  50,  80);
static const ImU32 kColAsymptote= IM_COL32(180, 100,   0,  90);
static const ImU32 kColDiagBg   = IM_COL32(  0,  15,   5,  80);
static const ImU32 kColEscape   = IM_COL32(220, 120,   0, 220);

// Format a duration in seconds as [H:]MM:SS
static void fmtTime(double totalSeconds, char* buf, std::size_t sz)
{
    if (totalSeconds < 0.0 || !std::isfinite(totalSeconds)) {
        std::snprintf(buf, sz, "--:--");
        return;
    }
    int t = static_cast<int>(totalSeconds + 0.5);
    int h = t / 3600; t %= 3600;
    int m = t / 60;   t %= 60;
    if (h > 0) std::snprintf(buf, sz, "%d:%02d:%02d", h, m, t);
    else        std::snprintf(buf, sz, "%02d:%02d",       m, t);
}

// ---------------------------------------------------------------------------
void OrbitalMFD::setContext(const char* refName, const char* frameName)
{
    m_refName   = refName;
    m_frameName = frameName;
}

// ---------------------------------------------------------------------------
// All orbital elements computed directly from r, v — no Kepler solver needed.
// This is safe for every eccentricity value.
void OrbitalMFD::update(const astro::PosState&     state,
                         const astro::EphemerisTime& /*et*/,
                         double                      mu,
                         double                      bodyRadiusKm)
{
    m_bodyRadiusKm = bodyRadiusKm;

    const double r   = glm::length(state.r);
    const double v2  = glm::dot(state.v, state.v);
    const double rv  = glm::dot(state.r, state.v);

    // Specific orbital energy (negative = bound, positive = escape).
    const double energy = 0.5 * v2 - mu / r;

    // Angular momentum vector and magnitude.
    const glm::dvec3 hVec = glm::cross(state.r, state.v);
    const double     h    = glm::length(hVec);

    // Eccentricity vector (points toward periapsis).
    const glm::dvec3 eVec = ((v2 - mu / r) * state.r - rv * state.v) / mu;
    const double     e    = glm::length(eVec);

    // Semi-major axis: negative for hyperbolic, huge for near-parabolic.
    const double a = (std::abs(energy) > 1.0e-12) ? -mu / (2.0 * energy) : 1.0e12;

    // Semi-latus rectum and periapsis/apoapsis distances.
    const double p  = h * h / mu;
    const double rp = p / (1.0 + e);
    const double ra = (e < 1.0) ? a * (1.0 + e) : -1.0;  // -1 = no apoapsis

    // Inclination (angle between h and ecliptic north).
    const double inc = (h > 1.0e-10)
        ? std::acos(std::clamp(hVec.z / h, -1.0, 1.0))
        : 0.0;

    // RAAN — angle of ascending node from +X (vernal equinox in ECLIPJ2000).
    const glm::dvec3 nVec = glm::cross(glm::dvec3(0.0, 0.0, 1.0), hVec);
    const double     nMag = glm::length(nVec);
    double Omega = 0.0;
    if (nMag > 1.0e-10) {
        Omega = std::atan2(nVec.y, nVec.x);
        if (Omega < 0.0) Omega += kTwoPi;
    }

    // Argument of periapsis.
    double w = 0.0;
    if (nMag > 1.0e-10 && e > kMinEcc) {
        w = std::acos(std::clamp(glm::dot(nVec / nMag, eVec / e), -1.0, 1.0));
        if (eVec.z < 0.0) w = kTwoPi - w;
    }

    // True anomaly from eccentricity vector — algebraic, works for all e.
    double nu       = 0.0;
    bool   trueValid = false;
    if (e > kMinEcc) {
        double cosNu = std::clamp(glm::dot(eVec / e, state.r / r), -1.0, 1.0);
        nu = std::acos(cosNu);
        if (rv < 0.0) nu = -nu;   // approaching periapsis → negative angle
        trueValid = std::isfinite(nu);
    }

    // Period (elliptic only).
    const double T = (e < 1.0 && a > 0.0)
        ? kTwoPi * std::sqrt(a * a * a / mu)
        : -1.0;

    // Time to apoapsis / periapsis — only for well-conditioned elliptic orbits.
    double tToApo = -1.0, tToPer = -1.0;
    if (trueValid && e >= kMinEcc && e < 0.9998 && T > 0.0) {
        double M = astro::OrbitElements::meanAnomalyFromTrueAnomaly(nu, e);
        if (std::isfinite(M)) {
            const double n     = kTwoPi / T;
            double       M_n   = std::fmod(M, kTwoPi);
            if (M_n < 0.0) M_n += kTwoPi;
            tToPer = std::fmod(kTwoPi - M_n, kTwoPi) / n;
            tToApo = std::fmod(kPi - M_n + kTwoPi, kTwoPi) / n;
        }
    }

    // Hyperbolic excess velocity v∞ = sqrt(μ/|a|).
    const bool isHyp = (e >= 1.0);
    const double vInf = isHyp ? std::sqrt(mu / std::abs(a)) : 0.0;

    // --- store ---
    m_altKm        = r  - bodyRadiusKm;
    m_velKms       = std::sqrt(v2);
    m_apoAltKm     = (ra > 0.0) ? ra - bodyRadiusKm : -1.0;
    m_perAltKm     = rp - bodyRadiusKm;
    m_incDeg       = inc  * kRad2Deg;
    m_raanDeg      = Omega * kRad2Deg;
    m_eccen        = e;
    m_argpeDeg     = w   * kRad2Deg;
    m_angMomKm2s   = h;
    m_smaKm        = a;
    m_periodMin    = (T  > 0.0) ? T / 60.0 : -1.0;
    m_trueDeg      = nu  * kRad2Deg;
    m_trueValid    = trueValid;
    m_tToApoSec    = tToApo;
    m_tToPerSec    = tToPer;
    m_isHyperbolic = isHyp;
    m_hypVinfKms   = vInf;

    // Freeze diagram scale for high-e / hyperbolic.
    if (e < kFreezeEcc && a > 0.0)
        m_frozenSmaKm = a;
}

// ---------------------------------------------------------------------------
void OrbitalMFD::renderDiagram(ImDrawList* dl,
                                ImVec2      diagOrigin,
                                float       diagSize) const
{
    const double e    = m_eccen;
    const bool   isHyp = m_isHyperbolic;

    const float pad    = 10.0f;
    const float usable = diagSize - 2.0f * pad;

    // Background.
    dl->AddRectFilled(diagOrigin,
                      { diagOrigin.x + diagSize, diagOrigin.y + diagSize },
                      kColDiagBg, 3.0f);

    // Scale reference: frozen SMA for high-e, current SMA otherwise.
    const double refA = (e >= kFreezeEcc && m_frozenSmaKm > 0.0)
                        ? m_frozenSmaKm : m_smaKm;
    if (refA <= 0.0 || usable <= 0.0) return;

    // sc (pixels per km): fit full ellipse or frozen extent.
    const double b_ref = (e < 1.0) ? refA * std::sqrt(std::max(0.0, 1.0 - e * e))
                                     : refA;  // for hyperbolic, just use a as reference
    const float sc = static_cast<float>(
        std::min((usable * 0.5) / refA, (usable * 0.5) / b_ref));

    if (isHyp) {
        // ---- Hyperbolic: focus-centred diagram --------------------------------
        const float cx = diagOrigin.x + diagSize * 0.5f;
        const float cy = diagOrigin.y + diagSize * 0.5f;

        const double a_abs = std::abs(m_smaKm);
        const double p     = a_abs * (e * e - 1.0);  // semi-latus rectum
        const double nu_inf = std::acos(-1.0 / e);   // asymptote angle

        // Asymptote lines (dim orange).
        {
            float extent = diagSize * 0.7f;
            float ax = static_cast<float>(std::cos(nu_inf));
            float ay = static_cast<float>(std::sin(nu_inf));
            dl->AddLine({ cx, cy }, { cx + ax * extent, cy - ay * extent }, kColAsymptote, 0.8f);
            dl->AddLine({ cx, cy }, { cx + ax * extent, cy + ay * extent }, kColAsymptote, 0.8f);
        }

        // Hyperbola near branch.
        constexpr int kSeg = 120;
        const double  nu0  = -(nu_inf - 0.04);
        ImVec2 pts[kSeg + 1];
        for (int i = 0; i <= kSeg; ++i) {
            double nu = nu0 + (nu_inf - 0.04) * 2.0 * i / kSeg;
            double r  = p / (1.0 + e * std::cos(nu));
            pts[i] = { cx + static_cast<float>(r * std::cos(nu)) * sc,
                       cy - static_cast<float>(r * std::sin(nu)) * sc };
        }
        dl->AddPolyline(pts, kSeg + 1, kColOrbit, ImDrawFlags_None, 1.0f);

        // Periapsis marker.
        {
            double rp = p / (1.0 + e);
            dl->AddCircle({ cx + static_cast<float>(rp) * sc, cy }, 3.0f, kColMark, 0, 1.5f);
        }

        // Central body.
        float bodyR = std::min(static_cast<float>(m_bodyRadiusKm) * sc, diagSize * 0.42f);
        bodyR = std::max(bodyR, 4.0f);
        dl->AddCircleFilled({ cx, cy }, bodyR, kColBody);
        dl->AddCircle      ({ cx, cy }, bodyR, kColBodyRim, 0, 1.0f);

        // Ship position on trajectory.
        if (m_trueValid && std::abs(m_trueDeg * kDeg2Rad) < nu_inf - 0.02) {
            double nu_ship = m_trueDeg * kDeg2Rad;
            double r_ship  = p / (1.0 + e * std::cos(nu_ship));
            float  sx = cx + static_cast<float>(r_ship * std::cos(nu_ship)) * sc;
            float  sy = cy - static_cast<float>(r_ship * std::sin(nu_ship)) * sc;
            dl->AddLine({ cx, cy }, { sx, sy }, kColShip, 1.0f);
            dl->AddCircleFilled({ sx, sy }, 4.0f, kColShip);
        }

    } else {
        // ---- Elliptic: ellipse-centred diagram --------------------------------
        const double a = refA;
        const double b = a * std::sqrt(std::max(0.0, 1.0 - e * e));

        // Ellipse centre at diagram centre.
        const float cx = diagOrigin.x + diagSize * 0.5f;
        const float cy = diagOrigin.y + diagSize * 0.5f;

        // Orbit ellipse.
        {
            const float sa = static_cast<float>(a) * sc;
            const float sb = static_cast<float>(b) * sc;
            constexpr int kSeg = 128;
            ImVec2 pts[kSeg + 1];
            for (int i = 0; i <= kSeg; ++i) {
                float t = static_cast<float>(kTwoPi) * i / kSeg;
                pts[i] = { cx + sa * std::cos(t), cy - sb * std::sin(t) };
            }
            dl->AddPolyline(pts, kSeg + 1, kColOrbit, ImDrawFlags_None, 1.0f);
        }

        // Apse line + markers.
        const float periPx = static_cast<float>(a) * sc;
        if (e > 0.005) {
            dl->AddLine({ cx - periPx, cy }, { cx + periPx, cy }, kColApseLine, 0.5f);
            dl->AddCircle({ cx + periPx, cy }, 3.0f, kColMark, 0, 1.5f);  // Pe
            dl->AddCircle({ cx - periPx, cy }, 2.5f, kColMark, 0, 1.0f);  // Ap
        }

        // Central body at the right focus.
        const float focusX = cx + static_cast<float>(a * e) * sc;
        {
            float bodyR = std::min(static_cast<float>(m_bodyRadiusKm) * sc,
                                   diagSize * 0.42f);
            bodyR = std::max(bodyR, 4.0f);
            dl->AddCircleFilled({ focusX, cy }, bodyR, kColBody);
            dl->AddCircle      ({ focusX, cy }, bodyR, kColBodyRim, 0, 1.0f);
        }

        // True anomaly line and ship dot.
        if (m_trueValid) {
            double nu    = m_trueDeg * kDeg2Rad;
            double r_orb = a * (1.0 - e * e) / (1.0 + e * std::cos(nu));
            float  sx = cx + static_cast<float>(r_orb * std::cos(nu) + a * e) * sc;
            float  sy = cy - static_cast<float>(r_orb * std::sin(nu))         * sc;
            dl->AddLine({ focusX, cy }, { sx, sy }, kColShip, 1.0f);
            dl->AddCircleFilled({ sx, sy }, 4.0f, kColShip);
        }
    }
}

// ---------------------------------------------------------------------------
void OrbitalMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const float lineH = ImGui::GetTextLineHeight() + 3.0f;
    const float pad   = 4.0f;

    // ---- Orbit diagram first (text layers on top) ----
    {
        const float diagSize = std::min(size.y, size.x);
        renderDiagram(dl, { origin.x + size.x - diagSize, origin.y }, diagSize);
    }

    // ---- Header ----
    {
        char buf[80];
        if (m_isHyperbolic)
            std::snprintf(buf, sizeof(buf), "%s  \xe2\x80\xa2  %s  \xe2\x80\xa2  HYP",
                          m_refName, m_frameName);
        else
            std::snprintf(buf, sizeof(buf), "%s  \xe2\x80\xa2  %s",
                          m_refName, m_frameName);
        ImU32 hdrCol = m_isHyperbolic ? kColEscape : kColDim;
        dl->AddText({ origin.x + pad, origin.y + 2.0f }, hdrCol, buf);
    }
    const float headerH = lineH + 2.0f;
    dl->AddLine({ origin.x,          origin.y + headerH },
                { origin.x + size.x, origin.y + headerH }, kColDivider, 0.5f);

    // ---- Two-column data rows ----
    const float colW = size.x * 0.5f;
    float       dy   = origin.y + headerH + pad;

    auto cell = [&](float colX, const char* label, const char* fmt, double val,
                    const char* unit = "") {
        dl->AddText({ colX + pad, dy }, kColGreen, label);
        char buf[48];
        std::snprintf(buf, sizeof(buf), fmt, val);
        std::strncat(buf, unit, sizeof(buf) - std::strlen(buf) - 1);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ colX + colW - tsz.x - pad, dy }, kColGreen, buf);
    };

    auto cellStr = [&](float colX, const char* label, const char* value,
                       ImU32 col = 0) {
        if (col == 0) col = kColGreen;
        dl->AddText({ colX + pad, dy }, col, label);
        ImVec2 tsz = ImGui::CalcTextSize(value);
        dl->AddText({ colX + colW - tsz.x - pad, dy }, col, value);
    };

    char tApBuf[16], tPeBuf[16];
    fmtTime(m_tToApoSec, tApBuf, sizeof(tApBuf));
    fmtTime(m_tToPerSec, tPeBuf, sizeof(tPeBuf));

    // Row 1: Alt / Vel
    cell(origin.x,        "Alt",  "%.1f",  m_altKm,    " km");
    cell(origin.x + colW, "Vel",  "%.3f",  m_velKms,   " km/s");   dy += lineH;

    // Row 2: Apo / Per
    if (m_apoAltKm >= 0.0)
        cell   (origin.x, "Apo", "%.1f", m_apoAltKm, " km");
    else
        cellStr(origin.x, "Apo", "\xe2\x88\x9e", kColEscape);       // ∞
    cell(origin.x + colW, "Per", "%.1f", m_perAltKm, " km");        dy += lineH;

    // Row 3: t-AP / t-PE  (hidden for hyperbolic)
    if (!m_isHyperbolic) {
        cellStr(origin.x,        "t-AP", tApBuf);
        cellStr(origin.x + colW, "t-PE", tPeBuf);                   dy += lineH;
    } else {
        // Show v∞ instead
        cell   (origin.x,        "v\xe2\x88\x9e", "%.3f", m_hypVinfKms, " km/s");
        dy += lineH;
    }

    // Row 4: Inc / RAAN
    cell(origin.x,        "Inc",  "%.2f\xc2\xb0", m_incDeg);
    cell(origin.x + colW, "RAAN", "%.2f\xc2\xb0", m_raanDeg);      dy += lineH;

    // Row 5: Ecc / ArgPe
    cell(origin.x,        "Ecc",  "%.5f",          m_eccen);
    cell(origin.x + colW, "ArgPe","%.2f\xc2\xb0",  m_argpeDeg);    dy += lineH;

    // Row 6: h / T (period only when bound)
    cell(origin.x, "h", "%.0f", m_angMomKm2s, " km\xc2\xb2/s");
    if (m_periodMin > 0.0)
        cell(origin.x + colW, "T", "%.2f", m_periodMin, " min");
    else
        cellStr(origin.x + colW, "T", "---");
}
