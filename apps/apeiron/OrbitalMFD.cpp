#include "OrbitalMFD.h"

#include <glm/glm.hpp>
#include <numbers>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

static constexpr double kRad2Deg = 180.0 / std::numbers::pi;
static constexpr double kDeg2Rad = std::numbers::pi / 180.0;

// ---- colours used both in render() and renderDiagram() --------------------
static const ImU32 kColGreen    = IM_COL32(  0, 210,  75, 210);
static const ImU32 kColDim      = IM_COL32(  0, 160,  55, 150);
static const ImU32 kColDivider  = IM_COL32(  0, 120,  40, 120);
static const ImU32 kColOrbit    = IM_COL32(  0, 200,  70, 200);
static const ImU32 kColBody     = IM_COL32( 30, 100, 200, 220);
static const ImU32 kColBodyRim  = IM_COL32( 60, 140, 255, 180);
static const ImU32 kColShip     = IM_COL32(220, 200,   0, 255);
static const ImU32 kColMark     = IM_COL32(  0, 180,  60, 160);
static const ImU32 kColApseLine = IM_COL32(  0, 140,  50,  80);
static const ImU32 kColDiagBg   = IM_COL32(  0,  15,   5,  80);

// ---------------------------------------------------------------------------
void OrbitalMFD::setContext(const char* refName, const char* frameName)
{
    m_refName   = refName;
    m_frameName = frameName;
}

// ---------------------------------------------------------------------------
void OrbitalMFD::update(const astro::PosState&     state,
                         const astro::EphemerisTime& et,
                         double                      mu,
                         double                      bodyRadiusKm)
{
    m_bodyRadiusKm = bodyRadiusKm;
    m_altKm  = glm::length(state.r) - bodyRadiusKm;
    m_velKms = glm::length(state.v);

    auto oe = astro::OrbitElements::fromStateVector(state, et, mu);

    m_apoAltKm   = oe.ap - bodyRadiusKm;
    m_perAltKm   = oe.rp - bodyRadiusKm;
    m_incDeg     = oe.i     * kRad2Deg;
    m_raanDeg    = oe.omega * kRad2Deg;
    m_eccen      = oe.e;
    m_argpeDeg   = oe.w     * kRad2Deg;
    m_angMomKm2s = oe.h;
    m_smaKm      = oe.a;
    m_periodMin  = (oe.T > 0.0) ? oe.T / 60.0 : 0.0;

    // True anomaly — undefined for near-circular orbits (M0 may be NaN).
    constexpr double kMinEcc = 1.0e-6;
    if (oe.e >= kMinEcc && std::isfinite(oe.M0)) {
        double ta = astro::OrbitElements::trueAnomalyFromMeanAnomaly(oe.M0, oe.e);
        if (std::isfinite(ta)) {
            m_trueDeg  = ta * kRad2Deg;
            m_trueValid = true;
            return;
        }
    }
    m_trueValid = false;
}

// ---------------------------------------------------------------------------
void OrbitalMFD::renderDiagram(ImDrawList* dl,
                                ImVec2      diagOrigin,
                                float       diagSize) const
{
    // Only draw for bound, non-degenerate orbits.
    if (m_eccen >= 1.0 || m_smaKm <= 0.0) return;

    const double a = m_smaKm;
    const double e = m_eccen;
    const double b = a * std::sqrt(std::max(0.0, 1.0 - e * e));

    const float pad    = 10.0f;
    const float usable = diagSize - 2.0f * pad;

    // Scale: fit the full ellipse (bounding box 2a × 2b, centred on ellipse centre).
    const float scaleA = (usable * 0.5f) / static_cast<float>(a);
    const float scaleB = (usable * 0.5f) / static_cast<float>(b);
    const float sc     = std::min(scaleA, scaleB);

    // Diagram centre = ellipse centre in screen space.
    const float cx = diagOrigin.x + diagSize * 0.5f;
    const float cy = diagOrigin.y + diagSize * 0.5f;

    // Background.
    dl->AddRectFilled(diagOrigin,
                      { diagOrigin.x + diagSize, diagOrigin.y + diagSize },
                      kColDiagBg, 3.0f);

    // Orbit ellipse — parametric, screen Y flipped.
    {
        const float sa = static_cast<float>(a) * sc;
        const float sb = static_cast<float>(b) * sc;
        constexpr int kSeg = 128;
        ImVec2 pts[kSeg + 1];
        for (int i = 0; i <= kSeg; ++i) {
            const float t = 2.0f * static_cast<float>(std::numbers::pi) * i / kSeg;
            pts[i] = { cx + sa * std::cos(t),
                       cy - sb * std::sin(t) };
        }
        dl->AddPolyline(pts, kSeg + 1, kColOrbit, ImDrawFlags_None, 1.0f);
    }

    // Apse line and markers (major axis: periapsis right, apoapsis left).
    const float periPx = static_cast<float>(a) * sc;   // +x from ellipse centre
    const float apoPx  = periPx;                         // −x from ellipse centre
    if (e > 0.005) {
        dl->AddLine({ cx - apoPx, cy }, { cx + periPx, cy }, kColApseLine, 0.5f);
        dl->AddCircle({ cx + periPx, cy }, 3.0f, kColMark, 0, 1.5f);  // periapsis
        dl->AddCircle({ cx - apoPx,  cy }, 3.0f, kColMark, 0, 1.0f);  // apoapsis
    }

    // Central body — focus is at (ae, 0) from ellipse centre (towards periapsis).
    const float focusX = cx + static_cast<float>(a * e) * sc;
    const float focusY = cy;
    {
        float bodyR = static_cast<float>(m_bodyRadiusKm) * sc;
        bodyR = std::min(bodyR, diagSize * 0.42f);  // cap so orbit ring stays visible
        bodyR = std::max(bodyR, 4.0f);
        dl->AddCircleFilled({ focusX, focusY }, bodyR, kColBody);
        dl->AddCircle      ({ focusX, focusY }, bodyR, kColBodyRim, 0, 1.0f);
    }

    // True-anomaly vector and ship position.
    if (m_trueValid) {
        const double nu = m_trueDeg * kDeg2Rad;
        const double r  = a * (1.0 - e * e) / (1.0 + e * std::cos(nu));

        // In ellipse-centre frame: x_ec = r*cos(nu) + ae,  y_ec = r*sin(nu)
        const float shipX = cx + static_cast<float>(r * std::cos(nu) + a * e) * sc;
        const float shipY = cy - static_cast<float>(r * std::sin(nu))         * sc;

        dl->AddLine({ focusX, focusY }, { shipX, shipY }, kColShip, 1.0f);
        dl->AddCircleFilled({ shipX, shipY }, 4.0f, kColShip);
    }
}

// ---------------------------------------------------------------------------
void OrbitalMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const float lineH = ImGui::GetTextLineHeight() + 3.0f;
    const float pad   = 4.0f;

    // ---- Header: reference body + frame --------------------------------
    {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "%s  \xe2\x80\xa2  %s", m_refName, m_frameName);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ origin.x + (size.x - tsz.x) * 0.5f, origin.y + 2.0f },
                    kColDim, buf);
    }
    const float headerH = lineH + 2.0f;
    dl->AddLine({ origin.x,          origin.y + headerH },
                { origin.x + size.x, origin.y + headerH }, kColDivider, 0.5f);

    // ---- Two-column data rows ------------------------------------------
    //  Column layout: | label  value | label  value |
    const float dataTop = origin.y + headerH + pad;
    const float colW    = size.x * 0.5f;

    // Helper: draw one label+value pair inside a column cell.
    auto cell = [&](float colX, float y, const char* label, const char* fmt, double val,
                    const char* unit = "") {
        dl->AddText({ colX + pad, y }, kColGreen, label);
        char buf[40];
        std::snprintf(buf, sizeof(buf), fmt, val);
        std::strncat(buf, unit, sizeof(buf) - std::strlen(buf) - 1);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        // Right-align value at the column's right edge with padding.
        dl->AddText({ colX + colW - tsz.x - pad, y }, kColGreen, buf);
    };

    float dy = dataTop;
    cell(origin.x,        dy, "Alt",  "%.1f",  m_altKm,      " km");
    cell(origin.x + colW, dy, "Vel",  "%.3f",  m_velKms,     " km/s");  dy += lineH;
    cell(origin.x,        dy, "Apo",  "%.1f",  m_apoAltKm,   " km");
    cell(origin.x + colW, dy, "Per",  "%.1f",  m_perAltKm,   " km");   dy += lineH;
    cell(origin.x,        dy, "Inc",  "%.2f\xc2\xb0", m_incDeg);
    cell(origin.x + colW, dy, "Ecc",  "%.5f",  m_eccen);               dy += lineH;
    cell(origin.x,        dy, "h",    "%.0f",  m_angMomKm2s, " km\xc2\xb2/s");
    if (m_periodMin > 0.0)
        cell(origin.x + colW, dy, "T", "%.1f", m_periodMin,  " min");  dy += lineH;

    // Thin divider above diagram.
    dl->AddLine({ origin.x,          dy },
                { origin.x + size.x, dy }, kColDivider, 0.5f);
    dy += 2.0f;

    // ---- Orbit diagram — square, centred horizontally ------------------
    const float diagH    = size.y - (dy - origin.y);   // remaining height
    const float diagSize = std::min(diagH, size.x);
    if (diagSize > 20.0f) {
        ImVec2 diagOrigin = {
            origin.x + (size.x - diagSize) * 0.5f,
            dy
        };
        renderDiagram(dl, diagOrigin, diagSize);
    }
}
