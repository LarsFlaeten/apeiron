#include "CislunarMFD.h"
#include "OrbitDiagram.h"

#include "apeiron/spacecraft/Autopilot.h"

#include <astro/SpiceCore.h>
#include <astro/ReferenceFrame.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <algorithm>
#include <numbers>
#include <vector>

static const astro::ReferenceFrame kEclipJ2000 =
    astro::ReferenceFrame::createEclipJ2000();

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

ImU32 CislunarMFD::dvColor(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    uint8_t r, g, b;
    if (t < 0.5f) {
        float u = t * 2.0f;
        r = static_cast<uint8_t>(255 * u); g = 210; b = 0;
    } else {
        float u = (t - 0.5f) * 2.0f;
        r = 255; g = static_cast<uint8_t>(210 * (1.0f - u)); b = 0;
    }
    return IM_COL32(r, g, b, 230);
}

static void fmtHMS(double sec, char* buf, int sz)
{
    if (sec < 0.0 || !std::isfinite(sec)) { std::snprintf(buf, sz, "--:--:--"); return; }
    int s  = static_cast<int>(sec + 0.5);
    int hh = s / 3600; s %= 3600;
    int mm = s / 60;   s %= 60;
    std::snprintf(buf, sz, "%d:%02d:%02d", hh, mm, s);
}

static void fmtTplus(double deltaSec, char* buf, int sz)
{
    const char sign = (deltaSec >= 0.0) ? '+' : '-';
    double abs_s = std::abs(deltaSec);
    if (abs_s >= 86400.0) {
        std::snprintf(buf, sz, "T%c%dd", sign, static_cast<int>(abs_s / 86400.0));
    } else {
        int mm = static_cast<int>(abs_s / 60.0);
        std::snprintf(buf, sz, "T%c%02d:%02d", sign, mm / 60, mm % 60);
    }
}

// ---------------------------------------------------------------------------
// update — called every frame
// ---------------------------------------------------------------------------
void CislunarMFD::update(const MFDContext& ctx)
{
    m_currentET   = ctx.currentEt.getETValue();
    m_shipR       = ctx.shipGeoR;
    m_shipV       = ctx.shipGeoV;
    m_mainThrustN = ctx.mainThrustN;
    m_shipMass    = ctx.shipMassKg;
    m_eventQueue  = ctx.eventQueue;
    m_autopilot   = ctx.autopilot;

    // Moon SOI detection — Moon SOI ~66183 km.
    try {
        astro::PosState moonState;
        astro::Spice().getRelativeGeometricState(301, 399, ctx.currentEt,
                                                 moonState, kEclipJ2000);
        glm::dvec3 moonPos(moonState.r.x, moonState.r.y, moonState.r.z);
        glm::dvec3 moonVel(moonState.v.x, moonState.v.y, moonState.v.z);
        double distFromMoon = glm::length(m_shipR - moonPos);
        m_inMoonSoi = (distFromMoon < 66200.0);
        if (m_inMoonSoi) {
            m_shipMoonR = m_shipR - moonPos;
            m_shipMoonV = m_shipV - moonVel;
        }
    } catch (...) {}

    m_burnCtrl.tick(m_currentET, m_shipR, m_shipV);
    if (m_inMoonSoi)
        m_loiBurnCtrl.tick(m_currentET, m_shipMoonR, m_shipMoonV);
}

// ---------------------------------------------------------------------------
// compute — run synchronous porkchop grid
// ---------------------------------------------------------------------------
void CislunarMFD::compute()
{
    if (m_computing) return;
    m_computing = true;
    m_hasData   = false;
    m_error.clear();

    m_params.centralBody          = 399;
    m_params.arrivalBody          = 301;
    m_params.departureBody        = 399;
    m_params.muCentral            = kGmEarth;
    m_params.useDepartureOverride = true;
    m_params.departureR           = m_shipR;
    m_params.departureV           = m_shipV;
    m_params.t0                   = m_currentET;
    m_params.t1                   = m_currentET + 90.0 * kDay;
    if (m_params.tofMin <= 0.0) m_params.tofMin = 3.0 * kDay;
    if (m_params.tofMax <= 0.0) m_params.tofMax = 8.0 * kDay;
    m_params.nDep = 90;
    m_params.nTof = 40;

    try {
        m_data    = spacecraft::computePorkchop(m_params);
        m_hasData = true;
        m_selDep  = -1;
        m_selTof  = -1;
    } catch (const std::exception& e) {
        m_error = e.what();
    } catch (...) {
        m_error = "unknown error";
    }

    if (m_hasData)
        computePlaneGrid();   // second pass: plane-change ΔV per cell

    m_computing = false;
}

// ---------------------------------------------------------------------------
// resolveSelected — recompute Lambert for the selected porkchop cell
// ---------------------------------------------------------------------------
void CislunarMFD::resolveSelected()
{
    m_detail = {};
    if (!m_hasData || m_selDep < 0 || m_selTof < 0) return;

    const double depET  = m_data.depET(m_selDep);
    const double tofSec = m_data.tofS(m_selTof);
    const double arrET  = depET + tofSec;

    // Departure state: ship's current parking orbit (same override used for grid).
    const glm::dvec3 depR = m_params.departureR;
    const glm::dvec3 depV = m_params.departureV;

    // Moon geocentric state at arrival.
    astro::PosState moonArr;
    try {
        astro::Spice().getRelativeGeometricState(
            301, 399, astro::EphemerisTime(arrET), moonArr, kEclipJ2000);
    } catch (...) { return; }

    glm::dvec3 moonArrR(moonArr.r.x, moonArr.r.y, moonArr.r.z);
    glm::dvec3 moonArrV(moonArr.v.x, moonArr.v.y, moonArr.v.z);

    glm::dvec3 vDep, vArr;
    if (!spacecraft::solveLambert(kGmEarth, depR, moonArrR, tofSec, true, vDep, vArr))
        return;

    m_detail.valid    = true;
    m_detail.depET    = depET;
    m_detail.arrET    = arrET;
    m_detail.tofSec   = tofSec;
    m_detail.dv1      = static_cast<float>(glm::length(vDep - depV));
    m_detail.dv2      = static_cast<float>(glm::length(vArr - moonArrV));
    m_detail.c3       = static_cast<float>(glm::dot(vDep - depV, vDep - depV));
    m_detail.depPos   = depR;
    m_detail.arrPos   = moonArrR;
    m_detail.vDep     = vDep;
    m_detail.vArr     = vArr;
    m_detail.vDepBody = depV;
    m_detail.vArrBody = moonArrV;
}

// ---------------------------------------------------------------------------
// computePlaneGrid — for each porkchop cell that has a valid Lambert solution,
// compute the plane-change ΔV required to align the current parking orbit with
// the departure hyperbola asymptote direction.
//
// Formula: planeErr = asin(|vInf · hHat| / |vInf|)
//          dvPC = 2 · v_orbit · sin(planeErr / 2)
//
// Requires a full second pass of Lambert solves (same N as the porkchop) so
// it roughly doubles the compute time.  Run once per porkchop computation.
// ---------------------------------------------------------------------------
void CislunarMFD::computePlaneGrid()
{
    m_planeGrid.clear();
    m_planeMin = 1e9f;
    m_planeMax = 0.0f;

    if (!m_hasData) return;

    const int nDep = m_data.params.nDep;
    const int nTof = m_data.params.nTof;
    m_planeGrid.assign(nDep * nTof, spacecraft::PorkchopData::kNoSolution);

    // Current parking orbit plane normal and orbital speed.
    const glm::dvec3 h = glm::cross(m_shipR, m_shipV);
    const double hMag = glm::length(h);
    if (hMag < 1e-6) return;  // degenerate — skip
    const glm::dvec3 hHat   = h / hMag;
    const double     vOrbit = glm::length(m_shipV);
    if (vOrbit < 0.01) return;

    const glm::dvec3 depR = m_params.departureR;
    const glm::dvec3 depV = m_params.departureV;

    for (int iDep = 0; iDep < nDep; ++iDep) {
        const double etDep = m_data.depET(iDep);
        for (int iTof = 0; iTof < nTof; ++iTof) {
            // Skip cells where the porkchop found no solution.
            if (m_data.get(m_data.dvTotal, iDep, iTof)
                    >= spacecraft::PorkchopData::kNoSolution * 0.5f)
                continue;

            const double tofSec = m_data.tofS(iTof);
            const double etArr  = etDep + tofSec;

            try {
                astro::PosState moonArr;
                astro::Spice().getRelativeGeometricState(
                    301, 399, astro::EphemerisTime(etArr), moonArr, kEclipJ2000);
                glm::dvec3 moonArrR(moonArr.r.x, moonArr.r.y, moonArr.r.z);

                glm::dvec3 vDep, vArr;
                if (!spacecraft::solveLambert(kGmEarth, depR, moonArrR,
                                              tofSec, true, vDep, vArr))
                    continue;

                const glm::dvec3 vInf    = vDep - depV;
                const double     vInfMag = glm::length(vInf);
                if (vInfMag < 1e-9) continue;

                const double sinPlane = std::abs(glm::dot(vInf, hHat)) / vInfMag;
                const double planeErr = std::asin(std::clamp(sinPlane, 0.0, 1.0));
                const double dvPC     = 2.0 * vOrbit * std::sin(planeErr * 0.5);
                const float  fPC      = static_cast<float>(dvPC);

                m_planeGrid[iTof * nDep + iDep] = fPC;
                m_planeMin = std::min(m_planeMin, fPC);
                m_planeMax = std::max(m_planeMax, fPC);
            } catch (...) {}
        }
    }
}

// ---------------------------------------------------------------------------
// computeTliIgnitionET — next prograde burn point where the departure
// hyperbola asymptote aligns with the required TLI departure velocity.
// ---------------------------------------------------------------------------
double CislunarMFD::computeTliIgnitionET() const
{
    if (!m_detail.valid) return 0.0;

    const double mu    = kGmEarth;
    const double rPark = kREarth + kParkAlts[m_parkIdx];

    glm::dvec3 h    = glm::cross(m_shipR, m_shipV);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) return 0.0;

    glm::dvec3 hHat  = h / hMag;
    glm::dvec3 ev    = glm::cross(m_shipV, h) / mu - glm::normalize(m_shipR);
    double     ecc   = glm::length(ev);
    if (ecc >= 1.0) return 0.0;

    glm::dvec3 periDir = (ecc > 1e-6) ? glm::normalize(ev) : glm::normalize(m_shipR);
    glm::dvec3 qDir    = glm::cross(hHat, periDir);
    double     p_orb   = hMag * hMag / mu;
    double     sma     = p_orb / (1.0 - ecc * ecc);
    if (sma <= 0.0) return 0.0;

    // Departure v∞ geocentric (= Lambert velocity − ship velocity at override).
    glm::dvec3 vInf     = m_detail.vDep - m_detail.vDepBody;
    double     vInfMag  = glm::length(vInf);
    if (vInfMag < 1e-6) return 0.0;

    // Eccentricity of departure hyperbola.
    const double e_hyp  = 1.0 + static_cast<double>(m_detail.c3) * rPark / mu;
    const double nu_inf = std::acos(-1.0 / e_hyp);  // asymptote TA

    // Project v∞ onto orbit plane.
    glm::dvec3 vInfProj    = vInf - glm::dot(vInf, hHat) * hHat;
    double     vInfProjMag = glm::length(vInfProj);
    if (vInfProjMag < 1e-9) return 0.0;

    // Burn TA: asymptote direction = burn TA + nu_inf.
    double phi_target = std::atan2(glm::dot(vInfProj, qDir),
                                   glm::dot(vInfProj, periDir));
    double burnTA = phi_target - nu_inf;
    while (burnTA >  std::numbers::pi) burnTA -= 2.0 * std::numbers::pi;
    while (burnTA < -std::numbers::pi) burnTA += 2.0 * std::numbers::pi;

    // Current TA.
    glm::dvec3 rHat  = glm::normalize(m_shipR);
    double ta_now = std::atan2(glm::dot(rHat, qDir), glm::dot(rHat, periDir));

    // Kepler time from ta_now to burnTA.
    auto meanAnom = [&](double ta) -> double {
        double E = 2.0 * std::atan2(
            std::sqrt(1.0 - ecc) * std::sin(ta * 0.5),
            std::sqrt(1.0 + ecc) * std::cos(ta * 0.5));
        return E - ecc * std::sin(E);
    };
    double orbPeriod  = 2.0 * std::numbers::pi * std::sqrt(sma * sma * sma / mu);
    double dM         = meanAnom(burnTA) - meanAnom(ta_now);
    if (dM < 0.0) dM += 2.0 * std::numbers::pi;
    double timeToBurn = dM / (2.0 * std::numbers::pi) * orbPeriod;

    // Burn duration (centred on burn TA).
    const double vCirc    = std::sqrt(mu / rPark);
    const double vPeri    = std::sqrt(static_cast<double>(m_detail.c3) + 2.0 * mu / rPark);
    const double dvTLI    = vPeri - vCirc;
    const double accel    = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
    const double burnDur  = dvTLI * 1000.0 / accel;

    return m_currentET + timeToBurn - burnDur * 0.5;
}

// ---------------------------------------------------------------------------
// Button labels
// ---------------------------------------------------------------------------
const char* CislunarMFD::leftLabel(int slot) const
{
    switch (m_page) {
    case 0:  // Window
        switch (slot) {
        case 0: return "DEP<";
        case 1: return "DEP>";
        case 2: return "TOF<";
        case 3: return "TOF>";
        case 4: return m_showPC ? "PC:ON" : "PC";
        case 5: return "RST";
        default: return "";
        }
    case 1:  // Plan
        if (slot == 3) return "ALT";
        if (slot == 4) return "BACK";
        return "";
    case 2:  // Burn
        if (slot == 2) {
            switch (m_burnCtrl.phase()) {
            case BurnPhase::Armed:
            case BurnPhase::PreIgnition:
            case BurnPhase::Executing: return "DSARM";
            default: return (m_detail.valid && glm::length(m_shipR) > 100.0) ? "ARM" : "";
            }
        }
        if (slot == 3) return "ALT";
        if (slot == 4) return "BACK";
        return "";
    case 3:  // Coast
        if (slot == 4) return "BACK";
        return "";
    case 4:  // LOI
        if (slot == 3) return "PE";
        if (slot == 4) return "BACK";
        return "";
    default: return "";
    }
}

const char* CislunarMFD::rightLabel(int slot) const
{
    switch (m_page) {
    case 0:
        if (slot == 0) return (m_selDep >= 0 && m_selTof >= 0 && m_hasData) ? "INFO" : "COMP";
        return "";
    case 1:
        if (slot == 4) return "BURN";
        return "";
    case 2:
        if (slot == 4) return "CST";
        return "";
    case 3:
        if (slot == 4) return "LOI";
        return "";
    case 4:
        if (slot == 4) {
            using BP = BurnPhase;
            const auto ph = m_loiBurnCtrl.phase();
            if (ph == BP::Armed || ph == BP::PreIgnition || ph == BP::Executing)
                return "DSARM";
            return (m_detail.valid && m_inMoonSoi) ? "ARM" : "";
        }
        return "";
    default: return "";
    }
}

// ---------------------------------------------------------------------------
// Button actions
// ---------------------------------------------------------------------------
void CislunarMFD::onLeft(int slot)
{
    switch (m_page) {
    case 0:
        switch (slot) {
        case 0: m_params.t0 -= 15.0*kDay; m_params.t1 -= 15.0*kDay; break;
        case 1: m_params.t0 += 15.0*kDay; m_params.t1 += 15.0*kDay; break;
        case 2: m_params.tofMin = std::max(1.0*kDay, m_params.tofMin - 6.0*3600.0); break;
        case 3: m_params.tofMax = std::min(30.0*kDay, m_params.tofMax + 6.0*3600.0); break;
        case 4: m_showPC = !m_showPC; break;
        case 5:
            m_params.t0     = m_currentET;
            m_params.t1     = m_currentET + 90.0*kDay;
            m_params.tofMin = 3.0*kDay;
            m_params.tofMax = 8.0*kDay;
            m_hasData = false; m_detail = {};
            break;
        default: break;
        }
        break;
    case 1:
        if (slot == 3) m_parkIdx = (m_parkIdx + 1) % static_cast<int>(std::size(kParkAlts));
        if (slot == 4) m_page = 0;
        break;
    case 2:
        if (slot == 2) {
            const auto ph = m_burnCtrl.phase();
            if (ph == BurnPhase::Armed || ph == BurnPhase::PreIgnition
                    || ph == BurnPhase::Executing) {
                m_burnCtrl.disarm();
            } else if (m_detail.valid && glm::length(m_shipR) > 100.0) {
                const double ignET = computeTliIgnitionET();
                if (ignET > m_currentET) {
                    const double mu    = kGmEarth;
                    const double rPark = kREarth + kParkAlts[m_parkIdx];
                    const double accel = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
                    const double vCirc = std::sqrt(mu / rPark);
                    const double vPeri = std::sqrt(static_cast<double>(m_detail.c3) + 2.0*mu/rPark);
                    const double dvTLI = vPeri - vCirc;
                    BurnPlan plan;
                    plan.name        = "TLI";
                    plan.ignitionET  = ignET;
                    plan.c3Required  = static_cast<double>(m_detail.c3);
                    plan.depBodyMu   = kGmEarth;
                    plan.dvMagnitude = dvTLI;
                    plan.burnDuration = dvTLI * 1000.0 / accel;
                    plan.retrogradeBurn = false;
                    plan.slewOnArm      = true;
                    if (m_eventQueue) {
                        // Replace the rough departure-epoch estimate with the
                        // orbit-accurate ignition time.
                        m_eventQueue->cancelByName("TLI");
                        m_eventQueue->schedule("TLI", ignET);
                    }
                    m_burnCtrl.arm(plan, m_autopilot, m_eventQueue);
                }
            }
        }
        if (slot == 3) m_parkIdx = (m_parkIdx + 1) % static_cast<int>(std::size(kParkAlts));
        if (slot == 4) m_page = 1;
        break;
    case 3:
        if (slot == 4) m_page = 2;
        break;
    case 4:
        if (slot == 3) m_loiPeIdx = (m_loiPeIdx + 1) % static_cast<int>(std::size(kLoiPeAlts));
        if (slot == 4) m_page = 3;
        break;
    default: break;
    }
}

void CislunarMFD::onRight(int slot)
{
    switch (m_page) {
    case 0:
        if (slot == 0) {
            if (m_selDep >= 0 && m_selTof >= 0 && m_hasData) {
                resolveSelected();
                if (m_detail.valid) {
                    m_page = 1;
                    if (m_eventQueue) {
                        // TLI: schedule at the porkchop departure epoch as a rough
                        // "warp to here" estimate.  ARM replaces it with the
                        // orbit-accurate ignition ET.
                        if (m_detail.depET > m_currentET)
                            m_eventQueue->schedule("TLI", m_detail.depET);
                        if (m_detail.arrET > m_currentET)
                            m_eventQueue->schedule("LOI", m_detail.arrET);
                        // MCC at midpoint of transfer.
                        const double mccET = m_detail.depET + m_detail.tofSec * 0.5;
                        if (mccET > m_currentET)
                            m_eventQueue->schedule("MCC", mccET);
                    }
                }
            } else {
                compute();
            }
        }
        break;
    case 1:
        if (slot == 4) m_page = 2;
        break;
    case 2:
        if (slot == 4) m_page = 3;
        break;
    case 3:
        if (slot == 4) m_page = 4;
        break;
    case 4:
        if (slot == 4) {
            using BP = BurnPhase;
            const auto ph = m_loiBurnCtrl.phase();
            if (ph == BP::Armed || ph == BP::PreIgnition || ph == BP::Executing) {
                m_loiBurnCtrl.disarm();
            } else if (m_detail.valid && m_inMoonSoi) {
                const double rPe   = kRMoon + kLoiPeAlts[m_loiPeIdx];
                const double vInf  = static_cast<double>(m_detail.dv2);
                const double vHyp  = std::sqrt(vInf*vInf + 2.0*kGmMoon/rPe);
                const double vCirc = std::sqrt(kGmMoon / rPe);
                const double dvLOI = vHyp - vCirc;
                const double accel = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;

                BurnPlan plan;
                plan.name           = "LOI";
                plan.ignitionET     = m_detail.arrET; // refined when in SOI
                plan.c3Required     = -kGmMoon / rPe;
                plan.depBodyMu      = kGmMoon;
                plan.dvMagnitude    = dvLOI;
                plan.burnDuration   = dvLOI * 1000.0 / accel;
                plan.retrogradeBurn = true;
                plan.slewOnArm      = false;
                if (m_eventQueue) m_eventQueue->cancelByName("LOI");
                m_loiBurnCtrl.arm(plan, m_autopilot, m_eventQueue);
            }
        }
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
void CislunarMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    if (m_page == 4) { renderLOI  (dl, origin, size); return; }
    if (m_page == 3) { renderCoast(dl, origin, size); return; }
    if (m_page == 2) { renderBurn (dl, origin, size); return; }
    if (m_page == 1) { renderPlan (dl, origin, size); return; }
    renderWindow(dl, origin, size);
}

// ---------------------------------------------------------------------------
// Page 0: porkchop window
// ---------------------------------------------------------------------------
void CislunarMFD::renderWindow(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kWhite  = IM_COL32(255, 255, 255, 220);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 220);
    const ImU32 kNoData = IM_COL32( 40,  40,  40, 200);
    const ImU32 kCyan   = IM_COL32( 80, 220, 255, 180);

    const float pad    = 4.0f;
    const float labelH = 11.0f;
    const float infoH  = labelH * 3.0f + labelH + pad;

    const float axisW = 22.0f;
    const float gx0 = origin.x + pad + axisW;
    const float gy0 = origin.y + pad;
    const float gx1 = origin.x + size.x - pad;
    const float gy1 = origin.y + size.y - infoH - pad;
    const float gw  = gx1 - gx0;
    const float gh  = gy1 - gy0;

    if (m_computing) {
        const char* msg = "COMPUTING...";
        ImVec2 tsz = ImGui::CalcTextSize(msg);
        dl->AddText({gx0+(gw-tsz.x)*0.5f, gy0+(gh-tsz.y)*0.5f}, kGreen, msg);
        return;
    }

    if (!m_hasData) {
        const char* l1 = "EARTH -> MOON  (90 day window)";
        const char* l2 = m_error.empty() ? "Press COMP to compute" : m_error.c_str();
        ImU32       c2 = m_error.empty() ? kDim : IM_COL32(255, 80, 80, 220);
        ImVec2 s1 = ImGui::CalcTextSize(l1), s2 = ImGui::CalcTextSize(l2);
        float cy = gy0 + (gh - labelH*2.0f - pad)*0.5f;
        dl->AddText({gx0+(gw-s1.x)*0.5f, cy},             kGreen, l1);
        dl->AddText({gx0+(gw-s2.x)*0.5f, cy+labelH+pad}, c2,     l2);
        return;
    }

    const int   nDep  = m_data.params.nDep;
    const int   nTof  = m_data.params.nTof;
    const float cellW = gw / static_cast<float>(nDep);
    const float cellH = gh / static_cast<float>(nTof);

    const float dvLo  = m_data.dvMin;
    const float dvHi  = std::min(m_data.dvMax, dvLo * 6.0f + 2.0f);
    const float dvRng = (dvHi > dvLo) ? (dvHi - dvLo) : 1.0f;

    // Grid cells.
    for (int iDep = 0; iDep < nDep; ++iDep) {
        for (int iTof = 0; iTof < nTof; ++iTof) {
            float dv  = m_data.get(m_data.dvTotal, iDep, iTof);
            float cx0 = gx0 + iDep * cellW;
            float cy0 = gy0 + (nTof - 1 - iTof) * cellH;
            ImU32 col = (dv >= spacecraft::PorkchopData::kNoSolution * 0.5f)
                      ? kNoData : dvColor((dv - dvLo) / dvRng);
            dl->AddRectFilled({cx0, cy0}, {cx0+cellW, cy0+cellH}, col);
        }
    }

    // Contour lines (marching squares).
    {
        const float range = dvHi - dvLo;
        float step = (range < 1.f) ? 0.1f : (range < 3.f) ? 0.25f
                   : (range < 6.f) ? 0.5f : 1.0f;
        std::vector<float> levs;
        for (float lev = std::ceil(dvLo/step)*step; lev <= dvHi && levs.size() < 12; lev += step)
            levs.push_back(lev);

        static const int8_t kMS[16][4] = {
            {-1,-1,-1,-1},{3,0,-1,-1},{0,1,-1,-1},{3,1,-1,-1},
            {1,2,-1,-1},  {0,0,0,0}, {0,2,-1,-1},{3,2,-1,-1},
            {2,3,-1,-1},  {2,0,-1,-1},{0,0,0,0}, {2,1,-1,-1},
            {1,3,-1,-1},  {1,0,-1,-1},{0,3,-1,-1},{-1,-1,-1,-1},
        };
        auto edgePt = [&](int i, int j, int edge, float lev,
                          float v0, float v1, float v2, float v3) -> ImVec2 {
            float bx=gx0+(i+0.5f)*cellW, by=gy0+(nTof-0.5f-j)*cellH;
            float tx=bx+cellW,            ty=by-cellH;
            switch(edge){
            case 0:{float t=(lev-v0)/(v1-v0);return{bx+t*cellW,by};}
            case 1:{float t=(lev-v1)/(v2-v1);return{tx,by-t*cellH};}
            case 2:{float t=(lev-v2)/(v3-v2);return{tx-t*cellW,ty};}
            case 3:{float t=(lev-v3)/(v0-v3);return{bx,ty+t*cellH};}
            default:return{bx,by};}
        };

        const float kNS = spacecraft::PorkchopData::kNoSolution * 0.5f;
        struct LabelPt { float x, y; bool valid=false; };
        std::vector<LabelPt> lblPts(levs.size());

        for (int li=0; li<static_cast<int>(levs.size()); ++li) {
            float lev=levs[li];
            for (int i=0;i<nDep-1;++i) for (int j=0;j<nTof-1;++j) {
                float v0=m_data.get(m_data.dvTotal,i,j),   v1=m_data.get(m_data.dvTotal,i+1,j);
                float v2=m_data.get(m_data.dvTotal,i+1,j+1),v3=m_data.get(m_data.dvTotal,i,j+1);
                if(v0>kNS||v1>kNS||v2>kNS||v3>kNS) continue;
                int ci=((v0>lev)?1:0)|((v1>lev)?2:0)|((v2>lev)?4:0)|((v3>lev)?8:0);
                if(ci==0||ci==15) continue;
                int8_t e[4];
                if(ci==5||ci==10){
                    bool ca=((v0+v1+v2+v3)*0.25f>lev);
                    if(ci==5){if(!ca){e[0]=3;e[1]=0;e[2]=1;e[3]=2;}else{e[0]=0;e[1]=1;e[2]=2;e[3]=3;}}
                    else     {if(!ca){e[0]=0;e[1]=1;e[2]=2;e[3]=3;}else{e[0]=3;e[1]=0;e[2]=1;e[3]=2;}}
                } else { e[0]=kMS[ci][0];e[1]=kMS[ci][1];e[2]=kMS[ci][2];e[3]=kMS[ci][3]; }
                if(e[0]>=0&&e[1]>=0){
                    ImVec2 pa=edgePt(i,j,e[0],lev,v0,v1,v2,v3);
                    ImVec2 pb=edgePt(i,j,e[1],lev,v0,v1,v2,v3);
                    dl->AddLine(pa,pb,IM_COL32(255,255,255,110),1.0f);
                    if(!lblPts[li].valid||i>(int)lblPts[li].x) lblPts[li]={float(i),(pa.y+pb.y)*0.5f,true};
                }
                if(e[2]>=0&&e[3]>=0){
                    ImVec2 pc=edgePt(i,j,e[2],lev,v0,v1,v2,v3);
                    ImVec2 pd=edgePt(i,j,e[3],lev,v0,v1,v2,v3);
                    dl->AddLine(pc,pd,IM_COL32(255,255,255,110),1.0f);
                }
            }
        }
        for (int li=0; li<static_cast<int>(levs.size()); ++li) {
            if(!lblPts[li].valid) continue;
            char buf[12]; std::snprintf(buf,sizeof(buf),"%.2g",levs[li]);
            ImVec2 tsz=ImGui::CalcTextSize(buf);
            float lx=gx0+(lblPts[li].x+1.5f)*cellW-tsz.x*0.5f, ly=lblPts[li].y-tsz.y*0.5f;
            dl->AddRectFilled({lx-1,ly-1},{lx+tsz.x+1,ly+tsz.y+1},IM_COL32(0,0,0,160));
            dl->AddText({lx,ly},IM_COL32(255,255,200,220),buf);
        }
    }

    // ---- Plane-change ΔV contours (cyan, shown when PC is toggled on) ----
    if (m_showPC && !m_planeGrid.empty()) {
        const float pcLo  = m_planeMin;
        const float pcHi  = m_planeMax;
        const float pcRng = (pcHi > pcLo) ? (pcHi - pcLo) : 1.0f;

        // Adaptive levels: 4-8 contour lines across the range.
        float step = pcRng / 6.0f;
        // Round to a "nice" step.
        const float steps[] = {0.01f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f};
        for (float s : steps) { if (s >= step) { step = s; break; } }
        std::vector<float> pcLevs;
        for (float lev=std::ceil(pcLo/step)*step; lev<=pcHi && pcLevs.size()<10; lev+=step)
            pcLevs.push_back(lev);

        static const int8_t kMS[16][4] = {
            {-1,-1,-1,-1},{3,0,-1,-1},{0,1,-1,-1},{3,1,-1,-1},
            {1,2,-1,-1},  {0,0,0,0}, {0,2,-1,-1},{3,2,-1,-1},
            {2,3,-1,-1},  {2,0,-1,-1},{0,0,0,0}, {2,1,-1,-1},
            {1,3,-1,-1},  {1,0,-1,-1},{0,3,-1,-1},{-1,-1,-1,-1},
        };
        auto pcEdgePt = [&](int i, int j, int edge, float lev,
                            float v0, float v1, float v2, float v3) -> ImVec2 {
            float bx=gx0+(i+0.5f)*cellW, by=gy0+(nTof-0.5f-j)*cellH;
            float tx=bx+cellW, ty=by-cellH;
            switch(edge){
            case 0:{float t=(lev-v0)/(v1-v0);return{bx+t*cellW,by};}
            case 1:{float t=(lev-v1)/(v2-v1);return{tx,by-t*cellH};}
            case 2:{float t=(lev-v2)/(v3-v2);return{tx-t*cellW,ty};}
            case 3:{float t=(lev-v3)/(v0-v3);return{bx,ty+t*cellH};}
            default:return{bx,by};}
        };

        const float kNS = spacecraft::PorkchopData::kNoSolution * 0.5f;
        struct LblPt { float x, y; bool valid=false; };
        std::vector<LblPt> pcLblPts(pcLevs.size());

        for (int li=0; li<static_cast<int>(pcLevs.size()); ++li) {
            float lev=pcLevs[li];
            // Colour: low plane-change = bright cyan, high = dim blue
            float t = std::clamp((lev-pcLo)/pcRng, 0.0f, 1.0f);
            uint8_t r = static_cast<uint8_t>(60  * (1.0f-t));
            uint8_t g = static_cast<uint8_t>(220 * (1.0f-t*0.5f));
            uint8_t b = 255;
            ImU32 pcCol = IM_COL32(r, g, b, 200);

            for (int i=0;i<nDep-1;++i) for (int j=0;j<nTof-1;++j) {
                const int idx00=j*nDep+i, idx10=j*nDep+i+1;
                const int idx11=(j+1)*nDep+i+1, idx01=(j+1)*nDep+i;
                float v0=m_planeGrid[idx00], v1=m_planeGrid[idx10];
                float v2=m_planeGrid[idx11], v3=m_planeGrid[idx01];
                if(v0>kNS||v1>kNS||v2>kNS||v3>kNS) continue;
                int ci=((v0>lev)?1:0)|((v1>lev)?2:0)|((v2>lev)?4:0)|((v3>lev)?8:0);
                if(ci==0||ci==15) continue;
                int8_t e[4];
                if(ci==5||ci==10){
                    bool ca=((v0+v1+v2+v3)*0.25f>lev);
                    if(ci==5){if(!ca){e[0]=3;e[1]=0;e[2]=1;e[3]=2;}else{e[0]=0;e[1]=1;e[2]=2;e[3]=3;}}
                    else     {if(!ca){e[0]=0;e[1]=1;e[2]=2;e[3]=3;}else{e[0]=3;e[1]=0;e[2]=1;e[3]=2;}}
                } else { e[0]=kMS[ci][0];e[1]=kMS[ci][1];e[2]=kMS[ci][2];e[3]=kMS[ci][3]; }
                if(e[0]>=0&&e[1]>=0){
                    ImVec2 pa=pcEdgePt(i,j,e[0],lev,v0,v1,v2,v3);
                    ImVec2 pb=pcEdgePt(i,j,e[1],lev,v0,v1,v2,v3);
                    dl->AddLine(pa,pb,pcCol,1.0f);
                    if(!pcLblPts[li].valid||i>(int)pcLblPts[li].x)
                        pcLblPts[li]={float(i),(pa.y+pb.y)*0.5f,true};
                }
                if(e[2]>=0&&e[3]>=0){
                    ImVec2 pc=pcEdgePt(i,j,e[2],lev,v0,v1,v2,v3);
                    ImVec2 pd=pcEdgePt(i,j,e[3],lev,v0,v1,v2,v3);
                    dl->AddLine(pc,pd,pcCol,1.0f);
                }
            }
        }
        // Labels for PC contours (at bottom of grid to avoid collision with dv labels).
        for (int li=0; li<static_cast<int>(pcLevs.size()); ++li) {
            if(!pcLblPts[li].valid) continue;
            char buf[10]; std::snprintf(buf,sizeof(buf),"%.2g",pcLevs[li]);
            ImVec2 tsz=ImGui::CalcTextSize(buf);
            float lx=gx0+(pcLblPts[li].x+0.5f)*cellW-tsz.x*0.5f;
            float ly=pcLblPts[li].y - tsz.y*0.5f;
            dl->AddRectFilled({lx-1,ly-1},{lx+tsz.x+1,ly+tsz.y+1},IM_COL32(0,0,30,180));
            dl->AddText({lx,ly},IM_COL32(100,230,255,230),buf);
        }
    }

    // Hover / click.
    const ImVec2 mp = ImGui::GetMousePos();
    const bool inGrid = mp.x>=gx0&&mp.x<gx1&&mp.y>=gy0&&mp.y<gy1;
    int hDep=-1, hTof=-1;
    if (inGrid) {
        hDep = std::clamp(static_cast<int>((mp.x-gx0)/cellW), 0, nDep-1);
        hTof = std::clamp(nTof-1-static_cast<int>((mp.y-gy0)/cellH), 0, nTof-1);
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) { m_selDep=hDep; m_selTof=hTof; }
    }
    if (m_selDep>=0&&m_selTof>=0) {
        float sx=gx0+(m_selDep+0.5f)*cellW, sy=gy0+(nTof-1-m_selTof+0.5f)*cellH;
        dl->AddLine({sx-5,sy},{sx+5,sy},kWhite,1.0f);
        dl->AddLine({sx,sy-5},{sx,sy+5},kWhite,1.0f);
    }

    // NOW marker.
    {
        const double t0=m_data.params.t0, t1=m_data.params.t1;
        if (m_currentET>=t0&&m_currentET<=t1) {
            float nx=gx0+static_cast<float>((m_currentET-t0)/(t1-t0))*gw;
            dl->AddLine({nx,gy0},{nx,gy1},kCyan,1.0f);
            const char* lbl="NOW";
            ImVec2 tsz=ImGui::CalcTextSize(lbl);
            dl->AddRectFilled({nx-1,gy1-tsz.y-3},{nx+tsz.x+2,gy1-1},IM_COL32(0,0,0,140));
            dl->AddText({nx+1,gy1-tsz.y-2},kCyan,lbl);
        }
    }

    // Departure axis ticks (bottom).
    for (int i=0;i<=4;++i) {
        float fx=gx0+gw*i/4.0f;
        double et=m_data.params.t0+(m_data.params.t1-m_data.params.t0)*i/4.0;
        std::string ts=astro::EphemerisTime(et).toISOUTCString(0);
        char buf[8]; std::snprintf(buf,sizeof(buf),"%.7s",ts.c_str());
        dl->AddLine({fx,gy1},{fx,gy1+3},kDim,1.0f);
        dl->AddText({fx-14,gy1+2},kDim,buf);
    }
    // TOF axis ticks (left).
    for (int i=0;i<=3;++i) {
        float fy=gy1-gh*i/3.0f;
        double tof=m_data.params.tofMin+(m_data.params.tofMax-m_data.params.tofMin)*i/3.0;
        int tofH=static_cast<int>(tof/3600.0+0.5);
        char buf[8]; std::snprintf(buf,sizeof(buf),"%dh",tofH);
        dl->AddLine({gx0-3,fy},{gx0,fy},kDim,1.0f);
        ImVec2 tsz=ImGui::CalcTextSize(buf);
        dl->AddText({gx0-tsz.x-2,fy-tsz.y*0.5f},kDim,buf);
    }

    // Info panel.
    const float infoY=gy1+labelH+pad;
    int di=inGrid?hDep:m_selDep, ti=inGrid?hTof:m_selTof;
    if (di>=0&&ti>=0) {
        double depET=m_data.depET(di), tofSec=m_data.tofS(ti);
        float dv1=m_data.get(m_data.dv1,di,ti), dv2=m_data.get(m_data.dv2,di,ti);
        float dvTot=m_data.get(m_data.dvTotal,di,ti);
        std::string depStr=astro::EphemerisTime(depET).toISOUTCString(0);
        int tofH=static_cast<int>(tofSec/3600.0+0.5);
        char buf[80];
        std::snprintf(buf,sizeof(buf),"DEP  %.10s  TOF %dh",depStr.c_str(),tofH);
        dl->AddText({origin.x+pad,infoY},kGreen,buf);
        if (dvTot<spacecraft::PorkchopData::kNoSolution*0.5f) {
            std::snprintf(buf,sizeof(buf),"TLI %.3f  v∞ %.3f  TOT %.3f km/s",dv1,dv2,dvTot);
            dl->AddText({origin.x+pad,infoY+labelH},kYellow,buf);
            // Plane-change ΔV for this cell (from precomputed grid).
            const int pidx = ti * nDep + di;
            const float kNS = spacecraft::PorkchopData::kNoSolution * 0.5f;
            if (!m_planeGrid.empty() && m_planeGrid[pidx] < kNS) {
                std::snprintf(buf,sizeof(buf),"PC  %.3f km/s  (total+PC %.3f)",
                              m_planeGrid[pidx], dvTot + m_planeGrid[pidx]);
                dl->AddText({origin.x+pad,infoY+2.0f*labelH},IM_COL32(100,230,255,230),buf);
            }
        } else {
            dl->AddText({origin.x+pad,infoY+labelH},kDim,"no solution");
        }
    } else {
        char buf[48]; std::snprintf(buf,sizeof(buf),"min ΔV %.3f km/s",m_data.dvMin);
        dl->AddText({origin.x+pad,infoY},kGreen,buf);
        if (!m_planeGrid.empty() && m_planeMin < 1e8f) {
            char buf2[48]; std::snprintf(buf2,sizeof(buf2),"PC range %.3f-%.3f km/s  [PC]",
                                         m_planeMin,m_planeMax);
            dl->AddText({origin.x+pad,infoY+labelH},IM_COL32(100,230,255,180),buf2);
        } else {
            dl->AddText({origin.x+pad,infoY+labelH},kDim,"click to select");
        }
    }
}

// ---------------------------------------------------------------------------
// Page 1: plan summary — dates, ΔV totals, LOI preview, orbit diagram
// ---------------------------------------------------------------------------
void CislunarMFD::renderPlan(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan   = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange = IM_COL32(255, 140,   0, 220);

    const float pad   = 4.0f;
    const float lineH = 11.0f;
    float y = origin.y + pad;
    const float cx = origin.x + pad;

    auto txt = [&](ImU32 col, const char* fmt, ...) {
        char buf[80]; va_list ap; va_start(ap,fmt);
        std::vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
        ImVec2 tsz=ImGui::CalcTextSize(buf);
        dl->AddRectFilled({cx-1,y-1},{cx+tsz.x+2,y+tsz.y+1},IM_COL32(0,0,0,160),2.0f);
        dl->AddText({cx,y},col,buf); y+=lineH;
    };

    if (!m_detail.valid) { txt(kDim,"No transfer selected"); return; }

    const double rPark    = kREarth + kParkAlts[m_parkIdx];
    const double vCirc    = std::sqrt(kGmEarth / rPark);
    const double vPeri    = std::sqrt(static_cast<double>(m_detail.c3) + 2.0*kGmEarth/rPark);
    const double dvTLI    = vPeri - vCirc;
    const double rPe      = kRMoon + kLoiPeAlts[m_loiPeIdx];
    const double vInfMoon = static_cast<double>(m_detail.dv2);
    const double vHyp     = std::sqrt(vInfMoon*vInfMoon + 2.0*kGmMoon/rPe);
    const double dvLOI    = vHyp - std::sqrt(kGmMoon/rPe);

    const double ignET = computeTliIgnitionET();
    char tIgnBuf[20] = "---";
    if (ignET > m_currentET) fmtTplus(ignET - m_currentET, tIgnBuf, sizeof(tIgnBuf));

    std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
    std::string arrStr = astro::EphemerisTime(m_detail.arrET).toISOUTCString(0);
    int tofH = static_cast<int>(m_detail.tofSec / 3600.0 + 0.5);

    txt(kGreen,  "TLI PLAN  EARTH -> MOON");
    y += 2;
    txt(kCyan,   "DEP  %.10s", depStr.c_str());
    txt(kCyan,   "ARR  %.10s", arrStr.c_str());
    txt(kCyan,   "TOF  %dh  (%.2f days)", tofH, m_detail.tofSec / kDay);
    y += 2;
    txt(kYellow, "Park alt   %.0f km  [ALT]", kParkAlts[m_parkIdx]);
    txt(kYellow, "TLI ΔV     %.3f km/s", dvTLI);
    txt(kCyan,   "v∞ Moon    %.3f km/s", vInfMoon);
    txt(kOrange, "LOI ΔV     %.3f km/s  (Pe +%.0f km)  [PE]", dvLOI, kLoiPeAlts[m_loiPeIdx]);
    y += 2;
    txt(kGreen,  "IGN        %s from now", tIgnBuf);

    // Orbit diagram: transfer arc + current ship position + Moon at arrival.
    const float diagH = origin.y + size.y - y - 4;
    if (diagH > 40) {
        OrbitDiagram diag;
        diag.setCentralBody({kREarth, IM_COL32(80,120,220,200)});
        if (m_detail.valid)
            diag.addOrbit(m_detail.depPos, m_detail.vDep, kGmEarth,
                         IM_COL32(60,200,255,180), "TLI", false, false);
        if (glm::length(m_shipR) > 100.0)
            diag.addOrbit(m_shipR, m_shipV, kGmEarth,
                         IM_COL32(0,210,75,200), "PARK", true, true);
        diag.addMarker(m_detail.arrPos, IM_COL32(200,200,200,220), "M");
        diag.render(dl, {origin.x, y}, {size.x, diagH}, &m_planViewRot);
    }
}

// ---------------------------------------------------------------------------
// Page 2: TLI burn execution — mirrors TransferMFD::renderDeparture
// ---------------------------------------------------------------------------
void CislunarMFD::renderBurn(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen   = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim     = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow  = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan    = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange  = IM_COL32(255, 140,   0, 220);
    const ImU32 kRed     = IM_COL32(255,  60,  60, 220);
    const ImU32 kBacking = IM_COL32(  0,   0,   0, 175);

    if (!m_detail.valid) {
        dl->AddText({origin.x+4, origin.y+4}, kDim, "No transfer selected");
        return;
    }

    const double rShip   = glm::length(m_shipR);
    const double vShipMag = glm::length(m_shipV);
    if (rShip < 100.0 || vShipMag < 0.01) {
        dl->AddText({origin.x+4, origin.y+4}, kDim, "No ship state");
        return;
    }

    // ---- Parking orbit geometry ----
    const double mu = kGmEarth;

    glm::dvec3 h    = glm::cross(m_shipR, m_shipV);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) {
        dl->AddText({origin.x+4, origin.y+4}, kDim, "Degenerate orbit");
        return;
    }

    glm::dvec3 hHat  = h / hMag;
    double     p_orb = hMag * hMag / mu;
    glm::dvec3 ev    = glm::cross(m_shipV, h) / mu - glm::normalize(m_shipR);
    double     ecc   = glm::length(ev);
    glm::dvec3 periDir = (ecc > 1e-6) ? glm::normalize(ev) : glm::normalize(m_shipR);
    glm::dvec3 qDir    = glm::cross(hHat, periDir);
    double     sma     = (ecc < 1.0) ? p_orb / (1.0 - ecc*ecc) : p_orb;
    double     altKm   = rShip - kREarth;

    // Ecliptic elements.
    double inc_cur  = std::acos(std::clamp(hHat.z, -1.0, 1.0)) * 180.0 / std::numbers::pi;
    double raan_cur = std::atan2(hHat.x, -hHat.y) * 180.0 / std::numbers::pi;
    if (raan_cur < 0.0) raan_cur += 360.0;
    double nodeLen  = std::sqrt(hHat.x*hHat.x + hHat.y*hHat.y);
    double argpe_cur = 0.0;
    if (nodeLen > 1e-9 && ecc > 1e-6) {
        glm::dvec3 nodeDir(-hHat.y/nodeLen, hHat.x/nodeLen, 0.0);
        argpe_cur = std::atan2(glm::dot(periDir, glm::cross(hHat, nodeDir)),
                               glm::dot(periDir, nodeDir)) * 180.0/std::numbers::pi;
        if (argpe_cur < 0.0) argpe_cur += 360.0;
    }

    glm::dvec3 rHat   = glm::normalize(m_shipR);
    double ta_now     = std::atan2(glm::dot(rHat, qDir), glm::dot(rHat, periDir));
    double taDeg_now  = ta_now * 180.0 / std::numbers::pi;
    if (taDeg_now < 0.0) taDeg_now += 360.0;

    // ---- V∞ ----
    glm::dvec3 vInf    = m_detail.vDep - m_detail.vDepBody;
    double     vInfMag = glm::length(vInf);
    if (vInfMag < 1e-6) {
        dl->AddText({origin.x+4, origin.y+4}, kDim, "Zero V-inf"); return;
    }
    glm::dvec3 vInfHat = vInf / vInfMag;

    // ---- Target plane (contains V∞) ----
    glm::dvec3 hProj   = hHat - glm::dot(hHat, vInfHat) * vInfHat;
    double     hProjL  = glm::length(hProj);
    glm::dvec3 hTarget = (hProjL > 1e-9) ? hProj/hProjL
                       : glm::normalize(glm::cross(vInfHat, glm::dvec3(0,0,1)));

    double inc_tgt  = std::acos(std::clamp(hTarget.z, -1.0, 1.0)) * 180.0/std::numbers::pi;
    double raan_tgt = std::atan2(hTarget.x, -hTarget.y) * 180.0/std::numbers::pi;
    if (raan_tgt < 0.0) raan_tgt += 360.0;
    double planeErr = std::asin(std::clamp(std::abs(glm::dot(vInfHat, hHat)), 0.0, 1.0))
                      * 180.0 / std::numbers::pi;

    // ---- AN / DN ----
    glm::dvec3 anVec = glm::cross(hTarget, hHat);
    double anLen = glm::length(anVec);
    bool hasNodes = (anLen > 1e-9);
    double ta_AN=0.0, ta_DN=0.0, taDeg_AN=0.0, taDeg_DN=0.0, tToAN=0.0, tToDN=0.0;
    glm::dvec3 anPos(0.0), dnPos(0.0);
    if (hasNodes) {
        glm::dvec3 anDir = anVec / anLen;
        ta_AN = std::atan2(glm::dot(anDir,qDir), glm::dot(anDir,periDir));
        ta_DN = ta_AN + std::numbers::pi;
        if (ta_DN > std::numbers::pi) ta_DN -= 2.0*std::numbers::pi;
        taDeg_AN = ta_AN*180.0/std::numbers::pi; if(taDeg_AN<0.0) taDeg_AN+=360.0;
        taDeg_DN = ta_DN*180.0/std::numbers::pi; if(taDeg_DN<0.0) taDeg_DN+=360.0;
        double rAN = p_orb/(1.0+ecc*std::cos(ta_AN));
        double rDN = p_orb/(1.0+ecc*std::cos(ta_DN));
        anPos = rAN*(std::cos(ta_AN)*periDir + std::sin(ta_AN)*qDir);
        dnPos = rDN*(std::cos(ta_DN)*periDir + std::sin(ta_DN)*qDir);
        if (ecc < 1.0 && sma > 0.0) {
            auto ta2M = [&](double ta) {
                double E = 2.0*std::atan2(std::sqrt(1.0-ecc)*std::sin(ta*0.5),
                                          std::sqrt(1.0+ecc)*std::cos(ta*0.5));
                return E - ecc*std::sin(E);
            };
            double n  = std::sqrt(mu/(sma*sma*sma));
            double Mn = ta2M(ta_now);
            double dAN = ta2M(ta_AN)-Mn; if(dAN<=0.0) dAN+=2.0*std::numbers::pi;
            double dDN = ta2M(ta_DN)-Mn; if(dDN<=0.0) dDN+=2.0*std::numbers::pi;
            tToAN = dAN/n; tToDN = dDN/n;
        }
    }

    // ---- Burn TA (asymptote alignment) ----
    const double rPark   = kREarth + kParkAlts[m_parkIdx];
    const double e_hyp   = 1.0 + static_cast<double>(m_detail.c3) * rPark / mu;
    const double nu_inf  = std::acos(-1.0 / e_hyp);
    glm::dvec3 vInfProj  = vInf - glm::dot(vInf, hHat) * hHat;
    double vInfProjMag   = glm::length(vInfProj);
    double burnTA=0.0, burnTADeg=0.0;
    if (vInfProjMag > 1e-9) {
        double phi_tgt = std::atan2(glm::dot(vInfProj,qDir), glm::dot(vInfProj,periDir));
        burnTA = phi_tgt - nu_inf;
        while (burnTA >  std::numbers::pi) burnTA -= 2.0*std::numbers::pi;
        while (burnTA < -std::numbers::pi) burnTA += 2.0*std::numbers::pi;
        burnTADeg = burnTA*180.0/std::numbers::pi; if(burnTADeg<0.0) burnTADeg+=360.0;
    }
    double rBurn   = p_orb/(1.0+ecc*std::cos(burnTA));
    double altBurn = rBurn - kREarth;
    glm::dvec3 burnPos = rBurn*(std::cos(burnTA)*periDir + std::sin(burnTA)*qDir);

    const double vCirc = std::sqrt(mu/rPark);
    const double vPeri = std::sqrt(static_cast<double>(m_detail.c3) + 2.0*mu/rPark);
    const double dvTMI = vPeri - vCirc;

    // ---- Burn timing ----
    double timeToBurnPoint=0.0, orbPeriod=0.0;
    if (ecc < 1.0 && sma > 0.0) {
        orbPeriod = 2.0*std::numbers::pi*std::sqrt(sma*sma*sma/mu);
        auto meanAnom = [&](double ta) {
            double E=2.0*std::atan2(std::sqrt(1.0-ecc)*std::sin(ta*0.5),
                                    std::sqrt(1.0+ecc)*std::cos(ta*0.5));
            return E-ecc*std::sin(E);
        };
        double dM = meanAnom(burnTA)-meanAnom(ta_now);
        if (dM < 0.0) dM += 2.0*std::numbers::pi;
        timeToBurnPoint = dM/(2.0*std::numbers::pi)*orbPeriod;
    }

    const double accelMs2    = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
    const double burnDuration = dvTMI * 1000.0 / accelMs2;
    const double timeToStartBurn = timeToBurnPoint - burnDuration * 0.5;

    // ---- Live remaining ΔV ----
    const double c3Now = vShipMag*vShipMag - 2.0*mu/rShip;
    const double c3Req = static_cast<double>(m_detail.c3);
    const double dvRemaining = (c3Now < c3Req)
        ? std::max(0.0, std::sqrt(c3Req + 2.0*mu/rShip) - vShipMag) : 0.0;
    const double burnRemaining = dvRemaining * 1000.0 / accelMs2;

    // ---- Orbit diagram — full area ----
    {
        OrbitDiagram diag;
        OrbitDiagram::Orbit o;
        o.r=m_shipR; o.v=m_shipV; o.mu=mu;
        o.colour=kGreen; o.showApses=(ecc>0.005); o.showCurrent=true;
        diag.addOrbit(o);

        // Ghost: target plane ring.
        {
            glm::dvec3 tPeri = glm::normalize(glm::cross(hTarget, hHat));
            if (glm::length(tPeri) < 0.5)
                tPeri = glm::normalize(glm::cross(hTarget, glm::dvec3(1,0,0)));
            OrbitDiagram::Orbit ot;
            ot.r=sma*tPeri; ot.v=std::sqrt(mu/sma)*glm::cross(hTarget,tPeri);
            ot.mu=mu; ot.colour=IM_COL32(60,180,255,60);
            ot.showApses=false; ot.showCurrent=false;
            diag.addOrbit(ot);
        }
        if (hasNodes) {
            diag.addMarker(anPos, IM_COL32(100,255,100,230), "AN");
            diag.addMarker(dnPos, IM_COL32(255,100,100,230), "DN");
        }
        diag.addMarker(burnPos, kYellow, "B");
        if (vInfProjMag > 1e-9)
            diag.addArrow({0.0,0.0,0.0}, vInfHat, 22.0f, kOrange);

        OrbitDiagram::CentralBody cb;
        cb.radiusKm=kREarth; cb.rimColour=IM_COL32(60,140,255,200);
        cb.axisColour=IM_COL32(60,140,255,100); cb.drawAxes=false;
        diag.setCentralBody(cb);
        diag.render(dl, origin, size, &m_burnViewRot);
    }

    // ---- Text overlay ----
    const float pad   = 3.0f;
    const float lineH = 11.0f;

    struct Line { ImU32 col; char txt[80]; };
    std::vector<Line> lines;
    auto add = [&](ImU32 col, const char* fmt, ...) {
        Line l; l.col=col;
        va_list ap; va_start(ap,fmt);
        std::vsnprintf(l.txt,sizeof(l.txt),fmt,ap); va_end(ap);
        lines.push_back(l);
    };
    auto sep = [&]() { add(0,""); };

    // Burn controller status.
    {
        const ImU32 kArmed=IM_COL32(255,220,0,240), kExec=IM_COL32(255,80,80,240), kDone=IM_COL32(0,210,75,240);
        switch (m_burnCtrl.phase()) {
        case BurnPhase::Armed: {
            char hms[16]; fmtHMS(m_burnCtrl.timeToIgnition(m_currentET),hms,sizeof(hms));
            add(kArmed,"ARMED  IGN T-%s  [DSARM]",hms); sep(); break;
        }
        case BurnPhase::PreIgnition: {
            double tti=m_burnCtrl.timeToIgnition(m_currentET);
            char hms[16];
            if(tti>=0.0) fmtHMS(tti,hms,sizeof(hms)); else std::snprintf(hms,sizeof(hms),"HOLD");
            add(kArmed,"PRE-IGN  T-%s  PROGRADE  [DSARM]",hms); sep(); break;
        }
        case BurnPhase::Executing: add(kExec,"EXECUTING BURN  [DSARM]"); sep(); break;
        case BurnPhase::Complete:  add(kDone,"BURN COMPLETE"); sep(); break;
        default: break;
        }
    }

    // Header.
    {
        std::string nowStr=astro::EphemerisTime(m_currentET).toISOUTCString(0);
        std::string depStr=astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
        char tp[16]; fmtTplus(m_detail.depET-m_currentET,tp,sizeof(tp));
        add(kDim,  "NOW  %.10s",nowStr.c_str());
        add(kGreen,"DEP  %.10s  (%s)",depStr.c_str(),tp);
    }
    add(kCyan,"V∞ %.3f km/s  C3 %.1f km²/s²",vInfMag,m_detail.c3);
    sep();

    // Current orbit.
    add(kCyan, "PARK ORBIT");
    add(kGreen," i %.1f  Ω %.1f  ω %.1f  e %.4f",inc_cur,raan_cur,argpe_cur,ecc);
    add(kGreen," TA %.1f  Alt %.0f km  SMA %.0f km",taDeg_now,altKm,sma);
    sep();

    // Plane error.
    const double vLEO = std::sqrt(mu/rShip);
    const double dvPC = 2.0*vLEO*std::sin(planeErr*std::numbers::pi/180.0*0.5);
    ImU32 tgtCol = (planeErr<0.5)?kGreen:(planeErr<5.0)?kOrange:kRed;
    add(kCyan, "TGT PLANE  i %.1f  Ω %.1f",inc_tgt,raan_tgt);
    add(tgtCol," Perr %.1f deg%s",planeErr,planeErr>5.0?" NEED CHG":"");
    if (planeErr > 0.05) {
        char buf[12]; fmtHMS(dvPC*1000.0/accelMs2,buf,sizeof(buf));
        add(tgtCol," dV-plane %.3f km/s  burn %s",dvPC,buf);
    }
    sep();

    // Nodes.
    add(kCyan,"NODES");
    if (hasNodes) {
        char bAN[12],bDN[12]; fmtHMS(tToAN,bAN,sizeof(bAN)); fmtHMS(tToDN,bDN,sizeof(bDN));
        add(IM_COL32(100,255,100,230)," AN  TA %.1f  T+ %s",taDeg_AN,bAN);
        add(IM_COL32(255,100,100,230)," DN  TA %.1f  T+ %s",taDeg_DN,bDN);
    } else { add(kDim," (coplanar)"); }
    sep();

    // Burn.
    add(kCyan,  "BURN  TA %.1f  Alt %.0f km",burnTADeg,altBurn);
    add(kDim,   "TLI  alt %.0f km  [ALT]",kParkAlts[m_parkIdx]);
    add(kYellow," dV-TLI %.3f km/s  (Vp %.3f  Vc %.3f)",dvTMI,vPeri,vCirc);
    {
        char bBurn[12],bStart[12],bToBurn[12];
        fmtHMS(burnDuration,bBurn,sizeof(bBurn));
        fmtHMS(timeToBurnPoint,bToBurn,sizeof(bToBurn));
        fmtHMS(std::max(0.0,timeToStartBurn),bStart,sizeof(bStart));
        add(kDim,   " accel %.3f m/s²  burn dur %s",accelMs2,bBurn);
        add(kGreen, " T-to-burn-point %s",bToBurn);
        add(kYellow," T-to-ignition   %s  (T-0.5burn)",bStart);
    }
    sep();
    {
        char bRem[12]; fmtHMS(burnRemaining,bRem,sizeof(bRem));
        ImU32 dvCol = (dvRemaining<0.001)?kGreen:(dvRemaining<dvTMI*0.1)?kYellow:kOrange;
        add(dvCol,"dV-REMAINING %.3f km/s  (%s)",dvRemaining,bRem);
        if (dvRemaining<0.001) add(kGreen," BURN COMPLETE");
    }

    // Draw text block.
    float maxW=0.0f;
    for (auto& l:lines) if(l.col) maxW=std::max(maxW,ImGui::CalcTextSize(l.txt).x);
    float bx0=origin.x+pad, by0=origin.y+pad;
    float bx1=bx0+maxW+pad*2.0f, by1=by0+static_cast<float>(lines.size())*lineH+pad;
    dl->AddRectFilled({bx0,by0},{bx1,by1},kBacking,3.0f);
    float tx=bx0+pad, ty=by0+pad*0.5f;
    for (auto& l:lines) { if(l.col) dl->AddText({tx,ty},l.col,l.txt); ty+=lineH; }
}

// ---------------------------------------------------------------------------
// Page 3: coast — transit progress
// ---------------------------------------------------------------------------
void CislunarMFD::renderCoast(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan   = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange = IM_COL32(255, 140,   0, 220);

    const float pad   = 4.0f;
    const float lineH = 11.0f;
    float y = origin.y + pad;
    const float cx = origin.x + pad;

    auto txt = [&](ImU32 col, const char* fmt, ...) {
        char buf[80]; va_list ap; va_start(ap,fmt);
        std::vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
        ImVec2 tsz=ImGui::CalcTextSize(buf);
        dl->AddRectFilled({cx-1,y-1},{cx+tsz.x+2,y+tsz.y+1},IM_COL32(0,0,0,160),2.0f);
        dl->AddText({cx,y},col,buf); y+=lineH;
    };

    txt(kGreen,"COAST  EARTH -> MOON");
    y += 4;

    if (!m_detail.valid) { txt(kDim,"No plan loaded"); return; }

    std::string depStr=astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
    std::string arrStr=astro::EphemerisTime(m_detail.arrET).toISOUTCString(0);
    txt(kCyan,"DEP  %.10s",depStr.c_str());
    txt(kCyan,"ARR  %.10s",arrStr.c_str());

    double progress = (m_detail.tofSec > 0.0)
        ? (m_currentET - m_detail.depET) / m_detail.tofSec : -1.0;

    if (progress >= 0.0 && progress <= 1.5) {
        char tp[16];
        fmtTplus(m_detail.arrET - m_currentET, tp, sizeof(tp));
        txt(kYellow,"Progress %.1f%%  (%s to arr)", progress*100.0, tp);
    }

    try {
        astro::PosState moonNow;
        astro::Spice().getRelativeGeometricState(301,399,
            astro::EphemerisTime(m_currentET), moonNow, kEclipJ2000);
        glm::dvec3 mp(moonNow.r.x,moonNow.r.y,moonNow.r.z);
        txt(kGreen,"Dist Moon  %.0f km",glm::length(m_shipR-mp));
    } catch (...) {}

    y += 4;
    if (m_inMoonSoi) txt(kOrange,"IN MOON SOI  -> LOI");
    else txt(kDim,"Outside Moon SOI");

    // Progress bar.
    if (progress >= 0.0) {
        float barX0=cx, barY0=y+4, barW=size.x-2*pad, barH=6.0f;
        float fill=std::clamp(static_cast<float>(progress),0.0f,1.0f)*barW;
        dl->AddRectFilled({barX0,barY0},{barX0+barW,barY0+barH},IM_COL32(40,40,40,200));
        dl->AddRectFilled({barX0,barY0},{barX0+fill,barY0+barH},IM_COL32(0,200,80,200));
        dl->AddRect      ({barX0,barY0},{barX0+barW,barY0+barH},IM_COL32(80,80,80,160));
        y = barY0+barH+4;
    }

    // Coast orbit diagram.
    const float diagH = origin.y+size.y-y-4;
    if (diagH > 40) {
        OrbitDiagram diag;
        diag.setCentralBody({kREarth, IM_COL32(80,120,220,200)});
        if (glm::length(m_shipR)>100.0)
            diag.addOrbit(m_shipR,m_shipV,kGmEarth,IM_COL32(200,200,80,180),"",false,true);
        if (m_detail.valid)
            diag.addOrbit(m_detail.depPos,m_detail.vDep,kGmEarth,
                         IM_COL32(80,180,80,120),"TLI",false,false);
        diag.addMarker(m_detail.arrPos,IM_COL32(200,200,200,220),"M");
        diag.render(dl,{origin.x,y},{size.x,diagH},&m_coastViewRot);
    }
}

// ---------------------------------------------------------------------------
// Page 4: LOI planning
// ---------------------------------------------------------------------------
void CislunarMFD::renderLOI(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan   = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange = IM_COL32(255, 140,   0, 220);
    const ImU32 kRed    = IM_COL32(255,  60,  60, 220);

    const float pad   = 4.0f;
    const float lineH = 11.0f;
    float y = origin.y + pad;
    const float cx = origin.x + pad;

    auto txt = [&](ImU32 col, const char* fmt, ...) {
        char buf[80]; va_list ap; va_start(ap,fmt);
        std::vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
        ImVec2 tsz=ImGui::CalcTextSize(buf);
        dl->AddRectFilled({cx-1,y-1},{cx+tsz.x+2,y+tsz.y+1},IM_COL32(0,0,0,160),2.0f);
        dl->AddText({cx,y},col,buf); y+=lineH;
    };

    txt(kGreen,"LOI  LUNAR ORBIT INSERT");
    y += 4;

    const double rPe   = kRMoon + kLoiPeAlts[m_loiPeIdx];
    const double vInf  = m_detail.valid ? static_cast<double>(m_detail.dv2) : 0.0;
    const double vHyp  = std::sqrt(vInf*vInf + 2.0*kGmMoon/rPe);
    const double vCirc = std::sqrt(kGmMoon / rPe);
    const double dvLOI = vHyp - vCirc;

    txt(kYellow,"Target Pe  +%.0f km  (r=%.0f km)  [PE]",kLoiPeAlts[m_loiPeIdx],rPe);
    txt(kYellow,"v∞ Moon    %.3f km/s",vInf);
    txt(kOrange,"LOI ΔV     %.3f km/s",dvLOI);
    y += 4;

    if (m_inMoonSoi) {
        double rNow=glm::length(m_shipMoonR), vNow=glm::length(m_shipMoonV);
        double C3  = vNow*vNow - 2.0*kGmMoon/rNow;
        txt(kCyan,"IN SOI  r=%.0f km  v=%.3f km/s",rNow,vNow);

        if (C3 > 0.0) {
            // Compute live approach Pe from angular momentum.
            glm::dvec3 hv = glm::cross(m_shipMoonR, m_shipMoonV);
            double hMag = glm::length(hv);
            double p    = hMag*hMag/kGmMoon;
            double eMag = std::sqrt(1.0 + C3*p/kGmMoon);
            double rPeActual = p/(1.0+eMag);
            double dvLOILive = std::sqrt(C3+2.0*kGmMoon/rPe) - std::sqrt(kGmMoon/rPe);
            txt(kGreen, "Hyp Pe  %.0f km  (alt +%.0f km)",rPeActual,rPeActual-kRMoon);
            txt(kOrange,"LOI ΔV (live)  %.3f km/s",dvLOILive);
        }
        y += 4;

        // Burn controller status.
        const auto ph = m_loiBurnCtrl.phase();
        if (ph != BurnPhase::Idle) {
            static const char* kPhN[]={"","ARMED","PRE-IGN","EXEC","DONE"};
            const ImU32 phCol=(ph==BurnPhase::Executing)?kRed:kYellow;
            txt(phCol,"LOI: %s",kPhN[static_cast<int>(ph)]);
            if (ph==BurnPhase::Armed||ph==BurnPhase::PreIgnition) {
                char hms[16]; fmtHMS(m_loiBurnCtrl.timeToIgnition(m_currentET),hms,sizeof(hms));
                txt(kYellow,"T-IGN  %s",hms);
            }
        } else {
            txt(kDim,"[ARM on right to execute]");
        }
    } else {
        txt(kDim,"Outside Moon SOI");
        txt(kDim,"Transit in progress...");
        if (m_detail.valid) {
            char tp[16]; fmtTplus(m_detail.arrET - m_currentET, tp, sizeof(tp));
            txt(kCyan,"LOI epoch  %s",tp);
        }
    }

    // Diagram.
    const float diagH = origin.y+size.y-y-4;
    if (diagH > 40) {
        OrbitDiagram diag;
        if (m_inMoonSoi) {
            diag.setCentralBody({kRMoon, IM_COL32(180,180,180,200)});
            if (glm::length(m_shipMoonR)>1.0) {
                diag.addOrbit(m_shipMoonR,m_shipMoonV,kGmMoon,
                             IM_COL32(255,160,60,200),"APPROACH",false,true);
                glm::dvec3 rHat2=glm::normalize(m_shipMoonR);
                diag.addOrbit(rHat2*rPe,
                              glm::cross(glm::dvec3(0,0,1),rHat2)*std::sqrt(kGmMoon/rPe),
                              kGmMoon, IM_COL32(80,200,80,120),"TGT",false,false);
            }
        } else {
            diag.setCentralBody({kREarth, IM_COL32(80,120,220,200)});
            if (m_detail.valid) {
                diag.addOrbit(m_detail.depPos,m_detail.vDep,kGmEarth,
                             IM_COL32(80,180,80,120),"TLI",false,false);
                diag.addMarker(m_detail.arrPos,IM_COL32(200,200,200,220),"M");
            }
        }
        diag.render(dl,{origin.x,y},{size.x,diagH},&m_loiViewRot);
    }
}
