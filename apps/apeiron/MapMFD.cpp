#include "MapMFD.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <numbers>

static constexpr double kPi    = std::numbers::pi;
static constexpr double kTwoPi = 2.0 * kPi;

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

glm::dvec2 MapMFD::toLatLon(const glm::dvec3& r)
{
    double rMag = glm::length(r);
    if (rMag < 1e-9) return { 0.0, 0.0 };
    double lat = std::asin(std::clamp(r.z / rMag, -1.0, 1.0));
    double lon = std::atan2(r.y, r.x);   // [-π, π]
    return { lat, lon };
}

ImVec2 MapMFD::project(double lat, double lon, ImVec2 origin, ImVec2 size)
{
    // Equirectangular: lon [-π, π] → x [0, w],  lat [-π/2, π/2] → y [h, 0]
    float x = origin.x + static_cast<float>((lon + kPi) / kTwoPi) * size.x;
    float y = origin.y + static_cast<float>(1.0 - (lat + kPi * 0.5) / kPi) * size.y;
    return { x, y };
}

glm::dvec3 MapMFD::rotZ(const glm::dvec3& v, double angle)
{
    double c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y,  s * v.x + c * v.y,  v.z };
}

// ---------------------------------------------------------------------------
// update() — recompute orbital elements and record trail point
// ---------------------------------------------------------------------------
void MapMFD::update(const glm::dvec3& pos,
                     const glm::dvec3& vel,
                     const Config&      cfg,
                     const glm::dmat3&  inertialToBody,
                     const glm::dvec3&  sunDirInertial)
{
    m_pos            = pos;
    m_vel            = vel;
    m_cfg            = cfg;
    m_inertialToBody = inertialToBody;
    m_sunDirInertial = sunDirInertial;
    m_valid          = false;

    const double mu = cfg.mu;
    const double r  = glm::length(pos);
    const double v2 = glm::dot(vel, vel);
    const double rv = glm::dot(pos, vel);
    if (r < 1e-6) return;

    // Angular momentum, eccentricity vector
    const glm::dvec3 hVec = glm::cross(pos, vel);
    const double     h    = glm::length(hVec);
    if (h < 1e-9) return;

    const glm::dvec3 eVec = ((v2 - mu / r) * pos - rv * vel) / mu;
    const double     e    = glm::length(eVec);

    const double energy = 0.5 * v2 - mu / r;
    const double a      = (std::abs(energy) > 1e-12) ? -mu / (2.0 * energy) : 1e12;

    m_ecc    = e;
    m_sma    = a;
    m_isHyp  = (e >= 1.0);
    m_p      = h * h / mu;
    m_period = (!m_isHyp && a > 0.0)
               ? kTwoPi * std::sqrt(a * a * a / mu) : 0.0;

    // Orbit frame
    m_normDir = glm::normalize(hVec);
    if (e > 1e-6) {
        m_periDir = glm::normalize(eVec);
    } else {
        // Circular: use current position as periDir placeholder
        glm::dvec3 rh = pos / r;
        rh -= glm::dot(rh, m_normDir) * m_normDir;
        m_periDir = (glm::length(rh) > 1e-6) ? glm::normalize(rh)
                                              : glm::dvec3(1, 0, 0);
    }
    m_qDir = glm::cross(m_normDir, m_periDir);

    // Current true anomaly and time from periapsis
    const double cosNu = std::clamp(glm::dot(m_periDir, pos / r), -1.0, 1.0);
    m_nuNow = std::acos(cosNu);
    if (rv < 0.0) m_nuNow = -m_nuNow;   // approaching periapsis

    m_tFromPe = 0.0;
    if (m_period > 0.0 && e < 0.9998) {
        // Elliptic: use Kepler's equation
        const double tanH = std::tan(m_nuNow * 0.5)
                          * std::sqrt((1.0 - e) / (1.0 + e));
        const double E = 2.0 * std::atan(tanH);
        const double M = E - e * std::sin(E);
        m_tFromPe = M / (kTwoPi / m_period);
    } else if (m_isHyp && e > 1.0) {
        // Hyperbolic
        const double k = std::sqrt((e - 1.0) / (e + 1.0));
        const double argF = k * std::tan(m_nuNow * 0.5);
        if (std::isfinite(argF) && std::abs(argF) < 1.0 - 1e-9) {
            const double F   = 2.0 * std::atanh(argF);
            const double M_h = e * std::sinh(F) - F;
            const double absA = std::abs(a);
            if (absA > 1.0)
                m_tFromPe = M_h / std::sqrt(mu / (absA * absA * absA));
        }
    }

    m_valid = true;

    // Record trail point: body-fixed lat/lon of current sub-satellite point
    const glm::dvec3 rBody = inertialToBody * pos;
    const glm::dvec2 ll    = toLatLon(rBody);
    m_trail.push_back(ll);
    // Prune trail: keep at most ¼ orbit (or 500 points for hyperbolic)
    const int maxPts = (m_period > 0.0)
        ? static_cast<int>(m_period * 0.25 / kTrailIntervalSec) + 2
        : 200;
    while (static_cast<int>(m_trail.size()) > std::max(maxPts, 10))
        m_trail.pop_front();
}

// ---------------------------------------------------------------------------
// drawTrack — render a lat/lon polyline, breaking at ±180° date-line wraps
// ---------------------------------------------------------------------------
void MapMFD::drawTrack(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                        const std::vector<glm::dvec2>& pts,
                        ImU32 col, float thickness) const
{
    if (pts.size() < 2) return;
    const float wrapThresh = static_cast<float>(size.x * 0.4f);

    ImVec2 prev = project(pts[0].x, pts[0].y, origin, size);
    for (std::size_t i = 1; i < pts.size(); ++i) {
        ImVec2 cur = project(pts[i].x, pts[i].y, origin, size);
        // Break the line if it crosses the date-line (large horizontal jump)
        if (std::abs(cur.x - prev.x) < wrapThresh)
            dl->AddLine(prev, cur, col, thickness);
        prev = cur;
    }
}

// ---------------------------------------------------------------------------
// buildForwardTrack — future orbit ground track
// Returns a vector of {lat, lon} body-fixed points.
// ---------------------------------------------------------------------------
static std::vector<glm::dvec2> buildForwardTrack(
    double ecc, double p, double nuStart, double tFromPeStart,
    double period, double mu, double sma,
    const glm::dvec3& periDir, const glm::dvec3& qDir,
    const glm::dmat3& inertialToBody, double rotRateRadSec,
    int numOrbits, int stepsPerOrbit, bool isHyp)
{
    std::vector<glm::dvec2> pts;
    if (isHyp) {
        // For hyperbolic: sweep from current ν to ~90% of asymptote ν
        const double nuAsym = std::acos(-1.0 / ecc) * 0.95;
        const int    nSteps = stepsPerOrbit;
        const double dNu    = (nuAsym - nuStart) / nSteps;
        if (dNu <= 0.0) return pts;
        pts.reserve(nSteps + 1);
        for (int k = 0; k <= nSteps; ++k) {
            const double nu  = nuStart + k * dNu;
            const double r_  = p / (1.0 + ecc * std::cos(nu));
            const glm::dvec3 rInertial = r_ * (std::cos(nu) * periDir
                                              + std::sin(nu) * qDir);
            // time from periapsis at this ν
            double dt = 0.0;
            const double ke = std::sqrt((ecc - 1.0) / (ecc + 1.0));
            const double argF = ke * std::tan(nu * 0.5);
            if (std::isfinite(argF) && std::abs(argF) < 1.0 - 1e-9) {
                const double F   = 2.0 * std::atanh(argF);
                const double M_h = ecc * std::sinh(F) - F;
                const double absA = std::abs(sma);
                if (absA > 1.0) {
                    dt = M_h / std::sqrt(mu / (absA * absA * absA))
                       - tFromPeStart;
                }
            }
            // Body rotation since t0
            const glm::dvec3 rBody = MapMFD::rotZ(inertialToBody * rInertial,
                                                   -rotRateRadSec * dt);
            pts.push_back(MapMFD::toLatLon(rBody));
        }
    } else {
        // Elliptic: sweep from current ν to current ν + numOrbits * 2π
        const double totalNu = numOrbits * kTwoPi;
        const int    nSteps  = stepsPerOrbit * numOrbits;
        const double dNu     = totalNu / nSteps;
        pts.reserve(nSteps + 1);
        const double n = kTwoPi / period;   // mean motion [rad/s]

        for (int k = 0; k <= nSteps; ++k) {
            const double nu = nuStart + k * dNu;
            const double r_ = p / (1.0 + ecc * std::cos(nu));
            const glm::dvec3 rInertial = r_ * (std::cos(nu) * periDir
                                              + std::sin(nu) * qDir);
            // Time from periapsis — must be continuous for multi-orbit tracks.
            // Normalize nu to [-π, π] so tan(nu/2) stays finite, then add
            // 2π * orbit-count (= floor((nu+π)/2π)) to keep M monotonic.
            const double nuNorm = std::remainder(nu, kTwoPi);  // in [-π, π]
            const double tanH   = std::tan(nuNorm * 0.5) * std::sqrt((1.0 - ecc) / (1.0 + ecc));
            const double E      = 2.0 * std::atan(tanH);
            const double M      = E - ecc * std::sin(E);
            const int    orb    = static_cast<int>(std::floor((nu + kPi) / kTwoPi));
            const double tAtNu  = (M + kTwoPi * orb) / n;
            const double dt     = tAtNu - tFromPeStart;

            const glm::dvec3 rBody = MapMFD::rotZ(inertialToBody * rInertial,
                                                   -rotRateRadSec * dt);
            pts.push_back(MapMFD::toLatLon(rBody));
        }
    }
    return pts;
}

// ---------------------------------------------------------------------------
// drawShadow — horizontal-strip shadow overlay
// For each latitude band: determine which longitude range is in shadow
// and draw a semi-transparent rectangle.
// ---------------------------------------------------------------------------
void MapMFD::drawShadow(ImDrawList* dl, ImVec2 origin, ImVec2 size) const
{
    const ImU32 kShadow = IM_COL32(0, 0, 0, 100);

    // Sun direction in body-fixed frame
    const glm::dvec3 sn = glm::normalize(m_inertialToBody * m_sunDirInertial);
    const double sx = sn.x, sy = sn.y, sz = sn.z;
    const double Rxy = std::sqrt(sx * sx + sy * sy);
    const double phi = std::atan2(sy, sx);   // sub-solar longitude

    const int nLat = 90;
    const float dLatPx = size.y / nLat;

    for (int ilat = 0; ilat < nLat; ++ilat) {
        // latitude range for this strip
        const double lat0 = (1.0 - (double)(ilat + 1) / nLat) * kPi - kPi * 0.5;
        const double lat1 = (1.0 - (double)ilat       / nLat) * kPi - kPi * 0.5;
        const double latM = (lat0 + lat1) * 0.5;
        const double clat = std::cos(latM), slat = std::sin(latM);

        // Shadow: clat * Rxy * cos(lon - phi) + slat * sz < 0
        const double A = clat * Rxy;
        const double B = slat * sz;

        const float stripY = origin.y + static_cast<float>(ilat) * dLatPx;
        const float stripH = dLatPx + 1.0f;  // +1 to avoid hairline gaps

        if (A < 1e-9) {
            // Near-polar: entire strip in shadow or entirely lit
            if (B < 0.0)
                dl->AddRectFilled({ origin.x, stripY },
                                  { origin.x + size.x, stripY + stripH }, kShadow);
            continue;
        }

        // cos(lon - phi) < -B/A  → shadow
        const double cosT = -B / A;

        if (cosT >= 1.0) {
            // Entirely in shadow
            dl->AddRectFilled({ origin.x, stripY },
                              { origin.x + size.x, stripY + stripH }, kShadow);
            continue;
        }
        if (cosT <= -1.0) {
            // Entirely lit — no shadow rect
            continue;
        }

        // Shadow arc centred at lon = phi + π, half-angle = acos(-cosT) = π - acos(cosT)
        const double dLon = std::acos(cosT);
        // Shadow spans lon ∈ (phi + dLon, phi + 2π - dLon)
        // which wraps around the anti-solar longitude phi + π.
        double lonL = phi + dLon;   // shadow starts here
        double lonR = phi + kTwoPi - dLon;  // shadow ends here
        // Normalise both to [-π, π]
        while (lonL >  kPi) lonL -= kTwoPi;
        while (lonL < -kPi) lonL += kTwoPi;
        while (lonR >  kPi) lonR -= kTwoPi;
        while (lonR < -kPi) lonR += kTwoPi;

        // Map longitudes to pixel x
        auto lonToX = [&](double lon) -> float {
            return origin.x + static_cast<float>((lon + kPi) / kTwoPi) * size.x;
        };

        const float xL = lonToX(lonL);
        const float xR = lonToX(lonR);

        if (xL < xR) {
            // Shadow is one contiguous strip [xL, xR] — no date-line wrap
            dl->AddRectFilled({ xL, stripY }, { xR, stripY + stripH }, kShadow);
        } else {
            // Shadow wraps across the date line: two rects [origin.x, xR] and [xL, end]
            dl->AddRectFilled({ origin.x, stripY }, { xR,             stripY + stripH }, kShadow);
            dl->AddRectFilled({ xL,       stripY }, { origin.x + size.x, stripY + stripH }, kShadow);
        }
    }
}

// ---------------------------------------------------------------------------
// drawTerminator
// ---------------------------------------------------------------------------
void MapMFD::drawTerminator(ImDrawList* dl, ImVec2 origin, ImVec2 size) const
{
    const ImU32 kTermCol = IM_COL32(255, 220, 80, 160);
    const glm::dvec3 sn = glm::normalize(m_inertialToBody * m_sunDirInertial);
    const double sx = sn.x, sy = sn.y, sz = sn.z;
    const double Rxy = std::sqrt(sx * sx + sy * sy);
    const double phi = std::atan2(sy, sx);

    // Build two terminator branches (one for each lon solution)
    std::vector<glm::dvec2> branch0, branch1;
    const int nLat = 120;
    branch0.reserve(nLat);
    branch1.reserve(nLat);

    for (int k = 0; k <= nLat; ++k) {
        const double lat = kPi * k / nLat - kPi * 0.5;
        const double clat = std::cos(lat), slat = std::sin(lat);
        const double A = clat * Rxy;
        if (std::abs(A) < 1e-9) continue;   // at poles: terminator is indeterminate
        const double cosT = -slat * sz / A;
        if (std::abs(cosT) > 1.0) continue; // entire latitude strip in shadow or lit
        const double dLon = std::acos(cosT);

        double lon0 = phi + dLon;
        double lon1 = phi - dLon;
        while (lon0 >  kPi) lon0 -= kTwoPi;
        while (lon0 < -kPi) lon0 += kTwoPi;
        while (lon1 >  kPi) lon1 -= kTwoPi;
        while (lon1 < -kPi) lon1 += kTwoPi;
        branch0.push_back({ lat, lon0 });
        branch1.push_back({ lat, lon1 });
    }
    drawTrack(dl, origin, size, branch0, kTermCol, 1.5f);
    drawTrack(dl, origin, size, branch1, kTermCol, 1.5f);
}

// ---------------------------------------------------------------------------
// render()
// ---------------------------------------------------------------------------
void MapMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kBg       = IM_COL32( 10,  20,  35, 220);
    const ImU32 kGrid     = IM_COL32( 30,  70,  90, 120);
    const ImU32 kGridMaj  = IM_COL32( 40, 100, 120, 160);
    const ImU32 kTrack    = IM_COL32(  0, 210,  75, 200);
    const ImU32 kFuture   = IM_COL32(  0, 210,  75, 130);
    const ImU32 kFuture2  = IM_COL32(  0, 160,  55,  80);
    const ImU32 kTrail    = IM_COL32(  0, 180,  60, 120);
    const ImU32 kShip     = IM_COL32(220, 200,   0, 255);
    const ImU32 kSun      = IM_COL32(255, 220,  60, 255);
    const ImU32 kDim      = IM_COL32(  0, 140,  50, 140);

    // Background: planet texture if available, dark fill otherwise
    if (m_mapTex) {
        dl->AddImage(m_mapTex,
                     origin, { origin.x + size.x, origin.y + size.y },
                     { 0.0f, 0.0f }, { 1.0f, 1.0f });
        // Dim overlay so grid/track colours pop against bright textures
        dl->AddRectFilled(origin, { origin.x + size.x, origin.y + size.y },
                          IM_COL32(0, 0, 0, 80));
    } else {
        dl->AddRectFilled(origin, { origin.x + size.x, origin.y + size.y }, kBg);
    }

    if (!m_valid) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim, "No orbital data");
        return;
    }

    // ---- Grid ----------------------------------------------------------------
    // Parallels every 30°
    for (int deg = -90; deg <= 90; deg += 30) {
        const double lat = deg * kPi / 180.0;
        ImVec2 p0 = project(lat, -kPi, origin, size);
        ImVec2 p1 = project(lat,  kPi, origin, size);
        ImU32 col = (deg == 0) ? kGridMaj : kGrid;
        dl->AddLine(p0, p1, col, (deg == 0) ? 1.2f : 0.7f);
    }
    // Meridians every 30°
    for (int deg = -180; deg < 180; deg += 30) {
        const double lon = deg * kPi / 180.0;
        ImVec2 p0 = project(-kPi * 0.5, lon, origin, size);
        ImVec2 p1 = project( kPi * 0.5, lon, origin, size);
        ImU32 col = (deg == 0 || deg == -180) ? kGridMaj : kGrid;
        dl->AddLine(p0, p1, col, (deg == 0 || deg == -180) ? 1.2f : 0.7f);
    }

    // ---- Shadow & terminator ------------------------------------------------
    drawShadow    (dl, origin, size);
    drawTerminator(dl, origin, size);

    // ---- Ground track -------------------------------------------------------
    const int stepsPerOrbit = 120;

    // Past trail
    {
        std::vector<glm::dvec2> trailPts(m_trail.begin(), m_trail.end());
        drawTrack(dl, origin, size, trailPts, kTrail, 1.5f);
    }

    // Future track — first orbit brighter, subsequent orbits dimmer
    if (m_period > 0.0 || m_isHyp) {
        // Orbit 1 (and 2, 3 for elliptic)
        const ImU32 cols[3] = { kTrack, kFuture, kFuture2 };
        const int   orbs    = m_isHyp ? 1 : m_showOrbits;

        for (int orb = 0; orb < orbs; ++orb) {
            // Starting ν for this orbit segment (each orbit shifted by 2π).
            // tFromPeStart is always m_tFromPe — the tAtNu formula inside
            // buildForwardTrack already adds period × orbit_count, so dt
            // (= tAtNu - m_tFromPe) correctly grows by one period per orbit.
            // Passing m_tFromPe + orb*period would cancel that offset and
            // make every orbit have the same body-rotation range (they'd overlap).
            const double nuStart = m_nuNow + orb * kTwoPi;
            auto seg = buildForwardTrack(
                m_ecc, m_p, nuStart, m_tFromPe,
                m_period, m_cfg.mu, m_sma,
                m_periDir, m_qDir,
                m_inertialToBody, m_cfg.rotRateRadSec,
                1, stepsPerOrbit, m_isHyp && orb == 0);
            drawTrack(dl, origin, size, seg, cols[orb], orb == 0 ? 1.5f : 1.0f);
            if (m_isHyp) break;
        }
    }

    // ---- Sub-solar point ----------------------------------------------------
    {
        const glm::dvec3 sunBody = glm::normalize(m_inertialToBody * m_sunDirInertial);
        const glm::dvec2 sunLL   = toLatLon(sunBody);
        ImVec2 sp = project(sunLL.x, sunLL.y, origin, size);
        dl->AddCircleFilled(sp, 4.0f, kSun);
        dl->AddCircle(sp, 6.0f, IM_COL32(255, 220, 60, 120), 12, 1.0f);
    }

    // ---- Sub-satellite point ------------------------------------------------
    {
        const glm::dvec3 rBody = m_inertialToBody * m_pos;
        const glm::dvec2 ll    = toLatLon(rBody);
        ImVec2 sp = project(ll.x, ll.y, origin, size);
        const float r = 5.0f;
        dl->AddLine({ sp.x - r, sp.y }, { sp.x + r, sp.y }, kShip, 1.5f);
        dl->AddLine({ sp.x, sp.y - r }, { sp.x, sp.y + r }, kShip, 1.5f);
        dl->AddCircle(sp, r + 1.5f, kShip, 12, 1.0f);
    }

    // ---- Text overlay -------------------------------------------------------
    {
        const float lh = 10.0f;
        float tx = origin.x + 3.0f, ty = origin.y + 3.0f;

        // Body name — top left
        if (!m_refName.empty())
            dl->AddText({ tx, ty }, IM_COL32(0, 200, 70, 200), m_refName.c_str());

        // Orbit info — top right
        char buf[40];
        const float rw = size.x - 3.0f;
        if (m_isHyp) {
            std::snprintf(buf, sizeof(buf), "HYP  e=%.4f", m_ecc);
        } else if (m_period > 0.0) {
            const int pMin = static_cast<int>(m_period / 60.0 + 0.5);
            std::snprintf(buf, sizeof(buf), "T=%d min", pMin);
        } else {
            std::snprintf(buf, sizeof(buf), "---");
        }
        ImVec2 bsz = ImGui::CalcTextSize(buf);
        dl->AddText({ origin.x + rw - bsz.x, ty }, kDim, buf);

        // TRK indicator — bottom right
        if (!m_isHyp) {
            std::snprintf(buf, sizeof(buf), "TRK %d", m_showOrbits);
            bsz = ImGui::CalcTextSize(buf);
            dl->AddText({ origin.x + rw - bsz.x,
                          origin.y + size.y - lh - 3.0f },
                        kDim, buf);
        }
    }
}

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------
const char* MapMFD::leftLabel(int slot) const
{
    if (slot == 0) return "TRK-";
    if (slot == 1) return "TRK+";
    return "";
}

void MapMFD::onLeft(int slot)
{
    if (slot == 0) m_showOrbits = std::max(1, m_showOrbits - 1);
    if (slot == 1) m_showOrbits = std::min(3, m_showOrbits + 1);
}

void MapMFD::update(const MFDContext& ctx)
{
    Config cfg;
    cfg.mu            = ctx.refBodyMu;
    cfg.radiusKm      = ctx.refBodyRadiusKm;
    cfg.rotRateRadSec = ctx.mapRotRateRadSec;
    update(ctx.shipRelRef.r, ctx.shipRelRef.v, cfg, ctx.inertialToBody, ctx.sunDirInertial);
}
