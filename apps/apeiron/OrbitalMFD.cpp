#include "OrbitalMFD.h"
#include "OrbitDiagram.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
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
static const ImU32 kColBody     = IM_COL32(  0,   0,   0,   0);  // REF body: transparent fill
static const ImU32 kColBodyRim  = IM_COL32(220,  50,  50, 200);  // REF body: red outline
static const ImU32 kColBodyAxis = IM_COL32(220,  50,  50, 130);  // REF body: red axes
static const ImU32 kColShip     = IM_COL32(220, 200,   0, 255);
static const ImU32 kColMark     = IM_COL32(  0, 180,  60, 160);
static const ImU32 kColApseLine = IM_COL32(  0, 140,  50,  80);
static const ImU32 kColAsymptote= IM_COL32(180, 100,   0,  90);
static const ImU32 kColDiagBg   = IM_COL32(  0,   0,   0,   0);  // fully transparent
static const ImU32 kColEscape   = IM_COL32(220, 120,   0, 220);
// Target colours (blue)
static const ImU32 kColTgtOrbit = IM_COL32( 60, 160, 255, 200);
static const ImU32 kColTgtMark  = IM_COL32( 60, 160, 255, 160);
static const ImU32 kColTgtShip  = IM_COL32( 60, 160, 255, 255);
static const ImU32 kColTgtLine  = IM_COL32( 40, 120, 200, 180);
static const ImU32 kColTgtText  = IM_COL32( 80, 180, 255, 220);

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
void OrbitalMFD::setFrames(std::vector<MFDFrame> frames)
{
    m_frames   = std::move(frames);
    m_frameIdx = 0;
}

// ---------------------------------------------------------------------------
std::string OrbitalMFD::consumePendingRef()
{
    std::string out;
    std::swap(out, m_pendingRef);
    return out;
}

// ---------------------------------------------------------------------------
void OrbitalMFD::setTargets(std::vector<std::string> names)
{
    m_tgtNames = std::move(names);
    m_tgtIdx   = -1;
    m_hasTgt   = false;
}

// ---------------------------------------------------------------------------
void OrbitalMFD::clearTarget()
{
    m_hasTgt = false;
}

// ---------------------------------------------------------------------------
// Compute and store target orbital elements (same logic as update()).
void OrbitalMFD::updateTarget(const astro::PosState& state, double mu, double bodyRadiusKm)
{
    m_tgtBodyRadKm = bodyRadiusKm;

    const astro::PosState& s = (!m_frames.empty())
        ? astro::PosState(m_frames[m_frameIdx].rot * state.r,
                          m_frames[m_frameIdx].rot * state.v)
        : state;

    const double r  = glm::length(s.r);
    const double v2 = glm::dot(s.v, s.v);
    const double rv = glm::dot(s.r, s.v);
    const double energy = 0.5 * v2 - mu / r;
    const glm::dvec3 hVec = glm::cross(s.r, s.v);
    const double     h    = glm::length(hVec);
    const glm::dvec3 eVec = ((v2 - mu / r) * s.r - rv * s.v) / mu;
    const double     e    = glm::length(eVec);
    const double a  = (std::abs(energy) > 1.0e-12) ? -mu / (2.0 * energy) : 1.0e12;
    const double p  = h * h / mu;
    const double rp = p / (1.0 + e);
    const double ra = (e < 1.0) ? a * (1.0 + e) : -1.0;

    const double inc = (h > 1.0e-10)
        ? std::acos(std::clamp(hVec.z / h, -1.0, 1.0)) : 0.0;
    const glm::dvec3 nVec = glm::cross(glm::dvec3(0.0, 0.0, 1.0), hVec);
    const double     nMag = glm::length(nVec);
    double Omega = 0.0;
    if (nMag > 1.0e-10) {
        Omega = std::atan2(nVec.y, nVec.x);
        if (Omega < 0.0) Omega += kTwoPi;
    }
    double w = 0.0;
    if (nMag > 1.0e-10 && e > kMinEcc) {
        w = std::acos(std::clamp(glm::dot(nVec / nMag, eVec / e), -1.0, 1.0));
        if (eVec.z < 0.0) w = kTwoPi - w;
    }
    double nu = 0.0; bool trueValid = false;
    if (e > kMinEcc) {
        double cosNu = std::clamp(glm::dot(eVec / e, s.r / r), -1.0, 1.0);
        nu = std::acos(cosNu);
        if (rv < 0.0) nu = -nu;
        trueValid = std::isfinite(nu);
    }
    const double T = (e < 1.0 && a > 0.0) ? kTwoPi * std::sqrt(a*a*a/mu) : -1.0;

    m_tgtAltKm     = r - bodyRadiusKm;
    m_tgtVelKms    = std::sqrt(v2);
    m_tgtApoKm     = (ra > 0.0) ? ra - bodyRadiusKm : -1.0;
    m_tgtPerKm     = rp - bodyRadiusKm;
    m_tgtIncDeg    = inc  * kRad2Deg;
    m_tgtRaanDeg   = Omega * kRad2Deg;
    m_tgtEccen     = e;
    m_tgtArgpeDeg  = w * kRad2Deg;
    m_tgtTrueDeg   = nu * kRad2Deg;
    m_tgtTrueValid = trueValid;
    m_tgtSmaKm     = a;
    m_tgtPeriodMin = (T > 0.0) ? T / 60.0 : -1.0;
    m_tgtHyp       = (e >= 1.0);

    if (h > 1.0e-10) {
        m_tgtNormDir = glm::normalize(hVec);
        if (e > kMinEcc) {
            m_tgtPeriDir = glm::normalize(eVec);
        } else {
            glm::dvec3 rHat = (r > 1.0e-6) ? (s.r / r) : glm::dvec3(1,0,0);
            rHat -= glm::dot(rHat, m_tgtNormDir) * m_tgtNormDir;
            m_tgtPeriDir = (glm::length(rHat) > 1.0e-6)
                ? glm::normalize(rHat)
                : glm::normalize(glm::cross(m_tgtNormDir, glm::dvec3(0,0,1)));
        }
        m_tgtQDir = glm::normalize(glm::cross(m_tgtNormDir, m_tgtPeriDir));
    }
    m_tgtShipDir = (r > 1.0e-6) ? glm::normalize(s.r) : m_tgtPeriDir;
    m_hasTgt = true;
}

// ---------------------------------------------------------------------------
const char* OrbitalMFD::leftLabel(int slot) const
{
    if (slot == 0) return "REF";
    if (slot == 1) return "FRM";
    if (slot == 2) return "TGT";
    return "";
}

// ---------------------------------------------------------------------------
void OrbitalMFD::onLeft(int slot)
{
    if (slot == 0) {
        m_refInputActive = !m_refInputActive;
        if (m_refInputActive)
            m_refInputBuf[0] = '\0';
    } else if (slot == 1) {
        if (!m_frames.empty()) {
            m_frameIdx    = (m_frameIdx + 1) % static_cast<int>(m_frames.size());
            m_diagViewRot = glm::dmat3(1.0);
        }
    } else if (slot == 2) {
        if (m_tgtNames.empty()) return;
        // Cycle: none(-1) → 0 → 1 → … → N-1 → none(-1)
        m_tgtIdx = (m_tgtIdx + 2) % (static_cast<int>(m_tgtNames.size()) + 1) - 1;
        if (m_tgtIdx < 0) m_hasTgt = false;
    }
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

    // Apply frame rotation so elements are measured in the selected frame.
    const astro::PosState& s = (!m_frames.empty())
        ? astro::PosState(m_frames[m_frameIdx].rot * state.r,
                          m_frames[m_frameIdx].rot * state.v)
        : state;

    const double r   = glm::length(s.r);
    const double v2  = glm::dot(s.v, s.v);
    const double rv  = glm::dot(s.r, s.v);

    // Specific orbital energy (negative = bound, positive = escape).
    const double energy = 0.5 * v2 - mu / r;

    // Angular momentum vector and magnitude.
    const glm::dvec3 hVec = glm::cross(s.r, s.v);
    const double     h    = glm::length(hVec);

    // Eccentricity vector (points toward periapsis).
    const glm::dvec3 eVec = ((v2 - mu / r) * s.r - rv * s.v) / mu;
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
        double cosNu = std::clamp(glm::dot(eVec / e, s.r / r), -1.0, 1.0);
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

    // --- 3D orbit geometry in the current frame ---
    // Always computed so the diagram can draw even circular orbits.
    if (h > 1.0e-10) {
        m_normDir3D = glm::normalize(hVec);
        if (e > kMinEcc) {
            m_periDir3D = glm::normalize(eVec);
        } else {
            // Circular: use current position as a stable reference direction.
            glm::dvec3 rHat = (r > 1.0e-6) ? (s.r / r) : glm::dvec3(1.0, 0.0, 0.0);
            // Project out the normal component so periDir lies in the orbit plane.
            rHat -= glm::dot(rHat, m_normDir3D) * m_normDir3D;
            m_periDir3D = (glm::length(rHat) > 1.0e-6)
                ? glm::normalize(rHat)
                : glm::normalize(glm::cross(m_normDir3D, glm::dvec3(0.0, 0.0, 1.0)));
        }
        m_qDir3D = glm::normalize(glm::cross(m_normDir3D, m_periDir3D));
    } else {
        m_periDir3D = {1.0, 0.0, 0.0};
        m_normDir3D = {0.0, 0.0, 1.0};
        m_qDir3D    = {0.0, 1.0, 0.0};
    }
    // Current spacecraft direction (used for ship dot on circular orbits).
    m_shipDir3D = (r > 1.0e-6) ? glm::normalize(s.r) : m_periDir3D;
}

// ---------------------------------------------------------------------------
// 3D orbit diagram.
//
// The central body sits at screen centre.  m_diagViewRot maps frame-space
// vectors to "view space".  Right-drag rotates; double-right-click resets.
// ---------------------------------------------------------------------------
void OrbitalMFD::renderDiagram(ImDrawList* dl, ImVec2 diagOrigin, float diagSize)
{
    if (m_smaKm == 0.0) return;

    // Reconstruct position from true anomaly or shipDir.
    const double e    = m_eccen;
    const bool   isHyp = m_isHyperbolic;
    const double slr  = isHyp
        ? std::abs(m_smaKm) * (e * e - 1.0)
        : m_smaKm           * (1.0 - e * e);

    const double rCur = m_trueValid
        ? slr / (1.0 + e * std::cos(m_trueDeg * kDeg2Rad))
        : (m_altKm + m_bodyRadiusKm);
    const glm::dvec3 shipPos = m_trueValid
        ? rCur * (std::cos(m_trueDeg * kDeg2Rad) * m_periDir3D
                + std::sin(m_trueDeg * kDeg2Rad) * m_qDir3D)
        : rCur * m_shipDir3D;

    OrbitDiagram diag;

    // Main orbit.  State at periapsis with mu=1:
    //   v_perp = sqrt((1+e)/rp)  → makes makeGeom reconstruct ecc = e exactly.
    {
        double rp = slr / (1.0 + e);
        OrbitDiagram::Orbit o;
        o.r           = rp * m_periDir3D;
        o.v           = std::sqrt((1.0 + e) / rp) * m_qDir3D;
        o.mu          = 1.0;
        o.colour      = kColOrbit;
        o.showApses   = (e > 0.005);
        o.showCurrent = false;   // ship dot added as separate marker
        o.segments    = 120;
        diag.addOrbit(o);
    }

    // Target orbit.
    if (m_hasTgt && m_tgtSmaKm != 0.0) {
        const double te   = m_tgtEccen;
        const bool   thy  = m_tgtHyp;
        const double tslr = thy
            ? std::abs(m_tgtSmaKm) * (te * te - 1.0)
            : m_tgtSmaKm           * (1.0 - te * te);
        double trp = tslr / (1.0 + te);
        OrbitDiagram::Orbit ot;
        ot.r         = trp * m_tgtPeriDir;
        ot.v         = std::sqrt((1.0 + te) / trp) * m_tgtQDir;
        ot.mu        = 1.0;
        ot.colour    = kColTgtOrbit;
        ot.showApses = (te > 0.005);
        diag.addOrbit(ot);

        // Target ship dot.
        glm::dvec3 tShipPos;
        if (m_tgtTrueValid) {
            double tnu = m_tgtTrueDeg * kDeg2Rad;
            double tr_ = tslr / (1.0 + te * std::cos(tnu));
            tShipPos = tr_ * (std::cos(tnu) * m_tgtPeriDir
                            + std::sin(tnu) * m_tgtQDir);
        } else {
            tShipPos = (m_tgtAltKm + m_tgtBodyRadKm) * m_tgtShipDir;
        }
        diag.addMarker(tShipPos, kColTgtShip, "");
    }

    // Central body.
    OrbitDiagram::CentralBody cb;
    cb.radiusKm   = m_bodyRadiusKm;
    cb.rimColour  = kColBodyRim;
    cb.axisColour = kColBodyAxis;
    cb.drawAxes   = true;
    diag.setCentralBody(cb);

    // Spacecraft marker.
    diag.addMarker(shipPos, kColShip, "");

    diag.render(dl, diagOrigin, {diagSize, diagSize}, &m_diagViewRot);
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

    // ---- REF input overlay (drawn first so other text appears on top) ----
    if (m_refInputActive) {
        ImGui::SetCursorScreenPos({ origin.x + pad, origin.y + pad });
        ImGui::SetNextItemWidth(size.x * 0.5f - pad * 2.0f);
        ImGui::PushStyleColor(ImGuiCol_FrameBg,     IM_COL32( 0, 40, 15, 200));
        ImGui::PushStyleColor(ImGuiCol_Text,         IM_COL32( 0,210, 75, 255));
        ImGui::PushStyleColor(ImGuiCol_Border,       IM_COL32( 0,210, 75, 180));
        bool entered = ImGui::InputText("##refInput", m_refInputBuf,
                                        sizeof(m_refInputBuf),
                                        ImGuiInputTextFlags_EnterReturnsTrue |
                                        ImGuiInputTextFlags_AutoSelectAll);
        ImGui::PopStyleColor(3);
        ImGui::SetKeyboardFocusHere(-1);  // keep focus
        if (entered && m_refInputBuf[0] != '\0') {
            m_pendingRef     = m_refInputBuf;
            m_refInputActive = false;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape))
            m_refInputActive = false;
    }

    // ---- Header ----
    {
        const char* frameName = (!m_frames.empty())
            ? m_frames[m_frameIdx].name : m_frameName;
        char buf[80];
        if (m_isHyperbolic)
            std::snprintf(buf, sizeof(buf), "%s  \xe2\x80\xa2  %s  \xe2\x80\xa2  HYP",
                          m_refName, frameName);
        else
            std::snprintf(buf, sizeof(buf), "%s  \xe2\x80\xa2  %s",
                          m_refName, frameName);
        ImU32 hdrCol = m_isHyperbolic ? kColEscape : kColDim;
        dl->AddText({ origin.x + pad, origin.y + 2.0f }, hdrCol, buf);

        // TGT label in header (right-aligned, blue)
        if (m_tgtIdx >= 0 && m_tgtIdx < static_cast<int>(m_tgtNames.size())) {
            char tgtBuf[48];
            std::snprintf(tgtBuf, sizeof(tgtBuf), "TGT: %s",
                          m_tgtNames[m_tgtIdx].c_str());
            ImVec2 tsz = ImGui::CalcTextSize(tgtBuf);
            dl->AddText({ origin.x + size.x - tsz.x - pad, origin.y + 2.0f },
                        kColTgtText, tgtBuf);
        }
    }
    const float headerH = lineH + 2.0f;
    dl->AddLine({ origin.x,          origin.y + headerH },
                { origin.x + size.x, origin.y + headerH }, kColDivider, 0.5f);

    // ---- Two text columns: ship (left) | target (right, blue) ----
    // Each column is 1/4 of the MFD width; diagram fills the middle+right half.
    const float colW  = size.x * 0.25f;
    float       dy    = origin.y + headerH + pad;

    // Ship column: label left-aligned, value right-aligned within colW.
    auto row = [&](const char* label, const char* fmt, double val,
                   const char* unit = "", ImU32 col = 0) {
        if (col == 0) col = kColGreen;
        dl->AddText({ origin.x + pad, dy }, col, label);
        char buf[48];
        std::snprintf(buf, sizeof(buf), fmt, val);
        std::strncat(buf, unit, sizeof(buf) - std::strlen(buf) - 1);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ origin.x + colW - tsz.x - pad, dy }, col, buf);
        dy += lineH;
    };
    auto rowStr = [&](const char* label, const char* value, ImU32 col = 0) {
        if (col == 0) col = kColGreen;
        dl->AddText({ origin.x + pad, dy }, col, label);
        ImVec2 tsz = ImGui::CalcTextSize(value);
        dl->AddText({ origin.x + colW - tsz.x - pad, dy }, col, value);
        dy += lineH;
    };

    // Target column: same layout but offset to second column.
    float dy2 = origin.y + headerH + pad;
    const float col2X = origin.x + colW + pad;
    const float col2R = origin.x + colW * 2.0f;
    auto rowTgt = [&](const char* label, const char* fmt, double val,
                      const char* unit = "") {
        dl->AddText({ col2X, dy2 }, kColTgtText, label);
        char buf[48];
        std::snprintf(buf, sizeof(buf), fmt, val);
        std::strncat(buf, unit, sizeof(buf) - std::strlen(buf) - 1);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ col2R - tsz.x, dy2 }, kColTgtText, buf);
        dy2 += lineH;
    };
    auto rowTgtStr = [&](const char* label, const char* value) {
        dl->AddText({ col2X, dy2 }, kColTgtText, label);
        ImVec2 tsz = ImGui::CalcTextSize(value);
        dl->AddText({ col2R - tsz.x, dy2 }, kColTgtText, value);
        dy2 += lineH;
    };

    char tApBuf[16], tPeBuf[16];
    fmtTime(m_tToApoSec, tApBuf, sizeof(tApBuf));
    fmtTime(m_tToPerSec, tPeBuf, sizeof(tPeBuf));

    // Ship rows
    row   ("Alt",   "%.1f",   m_altKm,     " km");
    row   ("Vel",   "%.3f",   m_velKms,    " km/s");
    if (m_apoAltKm >= 0.0)
        row("Apo",  "%.1f",   m_apoAltKm,  " km");
    else
        rowStr("Apo", "\xe2\x88\x9e", kColEscape);
    row   ("Per",   "%.1f",   m_perAltKm,  " km");
    if (!m_isHyperbolic) {
        rowStr("t-AP", tApBuf);
        rowStr("t-PE", tPeBuf);
    } else {
        row("v\xe2\x88\x9e", "%.3f", m_hypVinfKms, " km/s", kColEscape);
    }
    row   ("Inc",   "%.2f\xc2\xb0", m_incDeg);
    row   ("RAAN",  "%.2f\xc2\xb0", m_raanDeg);
    row   ("Ecc",   "%.5f",   m_eccen);
    row   ("ArgPe", "%.2f\xc2\xb0", m_argpeDeg);
    row   ("h",     "%.0f",   m_angMomKm2s, " km\xc2\xb2/s");
    if (m_periodMin > 0.0)
        row("T",    "%.2f",   m_periodMin,  " min");
    else
        rowStr("T", "---");

    // Target rows (only when a target is selected)
    if (m_hasTgt) {
        rowTgt   ("Alt",   "%.1f",   m_tgtAltKm,    " km");
        rowTgt   ("Vel",   "%.3f",   m_tgtVelKms,   " km/s");
        if (m_tgtApoKm >= 0.0)
            rowTgt("Apo",  "%.1f",   m_tgtApoKm,    " km");
        else
            rowTgtStr("Apo", "\xe2\x88\x9e");
        rowTgt   ("Per",   "%.1f",   m_tgtPerKm,    " km");
        dy2 += lineH * 2.0f;  // skip t-AP / t-PE rows (align with Inc)
        rowTgt   ("Inc",   "%.2f\xc2\xb0", m_tgtIncDeg);
        rowTgt   ("RAAN",  "%.2f\xc2\xb0", m_tgtRaanDeg);
        rowTgt   ("Ecc",   "%.5f",   m_tgtEccen);
        rowTgt   ("ArgPe", "%.2f\xc2\xb0", m_tgtArgpeDeg);
        rowTgt   ("",      "",       0.0);  // h row placeholder
        if (m_tgtPeriodMin > 0.0)
            rowTgt("T",    "%.2f",   m_tgtPeriodMin, " min");
        else
            rowTgtStr("T", "---");
    }
}
