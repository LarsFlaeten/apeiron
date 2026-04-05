#include "OrbitalMFD.h"

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
// vectors to "view space": view.x → screen right, view.y → screen up,
// view.z → toward viewer.  Default (identity) = looking down the frame +Z
// axis, so the reference plane (ecliptic / equatorial) fills the screen.
//
// Right-drag to rotate; double-right-click to reset.
// ---------------------------------------------------------------------------
void OrbitalMFD::renderDiagram(ImDrawList* dl, ImVec2 diagOrigin, float diagSize)
{
    const double e     = m_eccen;
    const bool   isHyp = m_isHyperbolic;
    const float  pad   = 8.0f;
    const float  usable = diagSize - 2.0f * pad;
    if (usable <= 0.0f) return;

    const float cx = diagOrigin.x + diagSize * 0.5f;
    const float cy = diagOrigin.y + diagSize * 0.5f;

    // ---- Scale ----------------------------------------------------------------
    // Use frozen SMA at high eccentricity to prevent runaway zoom.
    const double refA = (e >= kFreezeEcc && m_frozenSmaKm > 0.0)
                        ? m_frozenSmaKm : std::abs(m_smaKm);
    if (refA <= 0.0) return;

    // Maximum extent from the focus (central body) that must fit in 45% of usable.
    const double apoKm = isHyp
        ? (m_frozenSmaKm > 0.0 ? m_frozenSmaKm * 2.0 : std::abs(m_smaKm) * 2.0)
        : refA * (1.0 + std::min(e, kFreezeEcc));
    const float sc = static_cast<float>((usable * 0.45) / apoKm);

    // ---- Mouse rotation -------------------------------------------------------
    {
        const ImVec2 diagBR = { diagOrigin.x + diagSize, diagOrigin.y + diagSize };
        if (ImGui::IsMouseHoveringRect(diagOrigin, diagBR)) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                const ImVec2 d = ImGui::GetIO().MouseDelta;
                const double dx = d.x * 0.006;
                const double dy = d.y * 0.006;
                if (dx != 0.0 || dy != 0.0) {
                    const glm::dmat3 Rx(glm::rotate(glm::dmat4(1.0), dy, {1.0, 0.0, 0.0}));
                    const glm::dmat3 Ry(glm::rotate(glm::dmat4(1.0), dx, {0.0, 1.0, 0.0}));
                    m_diagViewRot = Rx * Ry * m_diagViewRot;
                }
            }
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right))
                m_diagViewRot = glm::dmat3(1.0);
        }
    }

    // ---- Helpers --------------------------------------------------------------
    // Project a 3D frame-space position to screen coords.
    auto proj = [&](const glm::dvec3& p) -> ImVec2 {
        const glm::dvec3 v = m_diagViewRot * p;
        return { cx + static_cast<float>(v.x * sc),
                 cy - static_cast<float>(v.y * sc) };
    };
    // View-space Z of a frame-space point (positive = facing viewer).
    auto viewZ = [&](const glm::dvec3& p) -> double {
        return (m_diagViewRot * p).z;
    };
    // Semi-latus rectum.
    const double slr = isHyp
        ? std::abs(m_smaKm) * (e * e - 1.0)
        : m_smaKm           * (1.0 - e * e);

    // ---- Reference-plane ring (frame XY plane, very dim) ----------------------
    {
        constexpr int kRSeg = 64;
        const double  rr    = apoKm * 0.80;
        ImVec2 pts[kRSeg + 1];
        for (int i = 0; i <= kRSeg; ++i) {
            const double t = kTwoPi * i / kRSeg;
            pts[i] = proj({ rr * std::cos(t), rr * std::sin(t), 0.0 });
        }
        dl->AddPolyline(pts, kRSeg + 1, IM_COL32(0, 70, 25, 45), ImDrawFlags_None, 0.5f);
    }

    // ---- Orbit curve ----------------------------------------------------------
    {
        constexpr int kSeg = 120;
        double nu0, nu1;
        if (isHyp) {
            const double nuInf = std::acos(-1.0 / e);
            nu0 = -(nuInf - 0.04);
            nu1 =  (nuInf - 0.04);
        } else {
            nu0 = 0.0;
            nu1 = kTwoPi;
        }
        ImVec2 pts[kSeg + 1];
        for (int i = 0; i <= kSeg; ++i) {
            const double nu = nu0 + (nu1 - nu0) * i / kSeg;
            const double r  = slr / (1.0 + e * std::cos(nu));
            pts[i] = proj(r * (std::cos(nu) * m_periDir3D + std::sin(nu) * m_qDir3D));
        }
        dl->AddPolyline(pts, kSeg + 1, kColOrbit, ImDrawFlags_None, 1.2f);
    }

    // ---- Asymptote lines (hyperbolic only) ------------------------------------
    if (isHyp) {
        const double nuInf = std::acos(-1.0 / e);
        const float  ext   = diagSize * 0.65f;
        for (int sign : {+1, -1}) {
            const glm::dvec3 dir3D = std::cos(nuInf) * m_periDir3D
                                   + sign * std::sin(nuInf) * m_qDir3D;
            const ImVec2 p0 = { cx, cy };
            const ImVec2 p1 = proj(dir3D * apoKm * 1.5);
            const float  dx = p1.x - p0.x;
            const float  dy = p1.y - p0.y;
            const float  len = std::sqrt(dx * dx + dy * dy);
            if (len > 0.5f)
                dl->AddLine(p0, { cx + dx / len * ext, cy + dy / len * ext },
                            kColAsymptote, 0.8f);
        }
    }

    // ---- Apse line + markers --------------------------------------------------
    if (e > 0.005) {
        const double rp3d = slr / (1.0 + e);
        const ImVec2 periPt = proj( rp3d * m_periDir3D);
        const ImVec2 focus  = { cx, cy };
        if (!isHyp) {
            const double ra3d  = slr / (1.0 - e);
            const ImVec2 apoPt = proj(-ra3d * m_periDir3D);
            dl->AddLine(periPt, apoPt, kColApseLine, 0.5f);
            dl->AddCircle(apoPt, 2.5f, kColMark, 0, 1.0f);
        } else {
            dl->AddLine(focus, periPt, kColApseLine, 0.5f);
        }
        dl->AddCircle(periPt, 3.0f, kColMark, 0, 1.5f);
    }

    // ---- Target orbit (drawn behind ship so ship dot is always on top) --------
    if (m_hasTgt) {
        const double te  = m_tgtEccen;
        const double ta  = m_tgtSmaKm;
        const bool   thy = m_tgtHyp;
        const double tslr = thy
            ? std::abs(ta) * (te * te - 1.0)
            : ta           * (1.0 - te * te);
        if (tslr > 0.0) {
            constexpr int kSeg = 120;
            double tnu0, tnu1;
            if (thy) {
                const double tnuInf = std::acos(-1.0 / te);
                tnu0 = -(tnuInf - 0.04); tnu1 = tnuInf - 0.04;
            } else { tnu0 = 0.0; tnu1 = kTwoPi; }
            ImVec2 tpts[kSeg + 1];
            for (int i = 0; i <= kSeg; ++i) {
                const double tnu = tnu0 + (tnu1 - tnu0) * i / kSeg;
                const double tr  = tslr / (1.0 + te * std::cos(tnu));
                tpts[i] = proj(tr * (std::cos(tnu) * m_tgtPeriDir
                                   + std::sin(tnu) * m_tgtQDir));
            }
            dl->AddPolyline(tpts, kSeg + 1, kColTgtOrbit, ImDrawFlags_None, 1.2f);
            // Apse line + markers
            if (te > 0.005) {
                const double trp = tslr / (1.0 + te);
                const ImVec2 tPeri = proj( trp * m_tgtPeriDir);
                if (!thy) {
                    const double tra  = tslr / (1.0 - te);
                    dl->AddLine(tPeri, proj(-tra * m_tgtPeriDir),
                                IM_COL32(40,120,200,60), 0.5f);
                    dl->AddCircle(proj(-tra * m_tgtPeriDir), 2.5f, kColTgtMark, 0, 1.0f);
                }
                dl->AddCircle(tPeri, 3.0f, kColTgtMark, 0, 1.5f);
            }
            // Target ship dot
            const double tRCur = m_tgtAltKm + m_tgtBodyRadKm;
            const ImVec2 focus = { cx, cy };
            if (m_tgtTrueValid) {
                const double tnu = m_tgtTrueDeg * kDeg2Rad;
                const double tr  = tslr / (1.0 + te * std::cos(tnu));
                const ImVec2 tShip = proj(tr * (std::cos(tnu) * m_tgtPeriDir
                                              + std::sin(tnu) * m_tgtQDir));
                dl->AddLine(focus, tShip, kColTgtLine, 1.0f);
                dl->AddCircleFilled(tShip, 4.0f, kColTgtShip);
            } else {
                const ImVec2 tShip = proj(tRCur * m_tgtShipDir);
                dl->AddLine(focus, tShip, kColTgtLine, 1.0f);
                dl->AddCircleFilled(tShip, 4.0f, kColTgtShip);
            }
        }
    }

    // ---- Central body: outline + axes in red, no fill -------------------------
    {
        const double bR = std::max(m_bodyRadiusKm, apoKm * 0.035);
        const ImU32 cFront = kColBodyRim;
        const ImU32 cBack  = IM_COL32(150, 30, 30, 50);

        // Outline circle (equatorial ring only — cleaner than 3 great circles)
        constexpr int kBSeg = 48;
        for (int i = 0; i < kBSeg; ++i) {
            const double t0 = kTwoPi * i       / kBSeg;
            const double t1 = kTwoPi * (i + 1) / kBSeg;
            const glm::dvec3 p0 = bR * glm::dvec3(std::cos(t0), std::sin(t0), 0.0);
            const glm::dvec3 p1 = bR * glm::dvec3(std::cos(t1), std::sin(t1), 0.0);
            const bool front = (viewZ(p0) + viewZ(p1)) >= 0.0;
            dl->AddLine(proj(p0), proj(p1), front ? cFront : cBack, 1.0f);
        }
        // Three axis stubs
        const float bPx = std::max(static_cast<float>(bR * sc), 4.0f);
        const float axLen = bPx * 1.6f;
        const ImVec2 centre = { cx, cy };
        // +X
        auto axEnd = [&](const glm::dvec3& dir) {
            const glm::dvec3 v = m_diagViewRot * dir;
            return ImVec2{ cx + static_cast<float>(v.x * sc) * 1.6f * static_cast<float>(bR),
                           cy - static_cast<float>(v.y * sc) * 1.6f * static_cast<float>(bR) };
        };
        (void)axLen;
        dl->AddLine(centre, axEnd({1,0,0}), kColBodyAxis, 1.0f);
        dl->AddLine(centre, axEnd({0,1,0}), kColBodyAxis, 1.0f);
        dl->AddLine(centre, axEnd({0,0,1}), kColBodyAxis, 1.0f);
    }

    // ---- Spacecraft position --------------------------------------------------
    {
        const ImVec2 focus = { cx, cy };
        if (m_trueValid) {
            const double nu    = m_trueDeg * kDeg2Rad;
            if (!isHyp || std::abs(nu) < std::acos(-1.0 / e) - 0.02) {
                const double r_s  = slr / (1.0 + e * std::cos(nu));
                const ImVec2 shipPt = proj(r_s * (std::cos(nu) * m_periDir3D
                                                 + std::sin(nu) * m_qDir3D));
                dl->AddLine(focus, shipPt, kColShip, 1.0f);
                dl->AddCircleFilled(shipPt, 4.0f, kColShip);
            }
        } else {
            // Circular orbit: use current spacecraft direction.
            const double rCur = m_altKm + m_bodyRadiusKm;
            const ImVec2 shipPt = proj(rCur * m_shipDir3D);
            dl->AddLine(focus, shipPt, kColShip, 1.0f);
            dl->AddCircleFilled(shipPt, 4.0f, kColShip);
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
