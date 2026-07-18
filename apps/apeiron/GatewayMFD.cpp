#include "GatewayMFD.h"
#include "OrbitDiagram.h"

#include "apeiron/spacecraft/Autopilot.h"
#include "VoiceAnnouncer.h"

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
ImU32 GatewayMFD::dvColor(float t)
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
void GatewayMFD::update(const MFDContext& ctx)
{
    m_currentET   = ctx.currentEt.getETValue();
    m_shipR       = ctx.shipGeoR;
    m_shipV       = ctx.shipGeoV;
    m_mainThrustN = ctx.mainThrustN;
    m_shipMass    = ctx.shipMassKg;
    m_eventQueue  = ctx.eventQueue;
    m_autopilot   = ctx.autopilot;

    // Moon SOI detection.
    m_gatewayOk = false;
    try {
        astro::PosState moonState;
        astro::Spice().getRelativeGeometricState(301, 399, ctx.currentEt,
                                                 moonState, kEclipJ2000);
        const glm::dvec3 moonPos(moonState.r.x, moonState.r.y, moonState.r.z);
        const glm::dvec3 moonVel(moonState.v.x, moonState.v.y, moonState.v.z);
        const double distFromMoon = glm::length(m_shipR - moonPos);
        m_inMoonSoi = (distFromMoon < 66200.0);
        if (m_inMoonSoi) {
            m_shipMoonR = m_shipR - moonPos;
            m_shipMoonV = m_shipV - moonVel;

            // Compute Gateway's Moon-centric state.
            if (m_gatewayConfig && m_gatewayConfig->hasOrbit) {
                astro::PosState gwState =
                    spacecraft::vehicleStateAtEt(*m_gatewayConfig, ctx.currentEt);
                m_gatewayMoonR = glm::dvec3(gwState.r.x - moonState.r.x,
                                             gwState.r.y - moonState.r.y,
                                             gwState.r.z - moonState.r.z);
                m_gatewayMoonV = glm::dvec3(gwState.v.x - moonState.v.x,
                                             gwState.v.y - moonState.v.y,
                                             gwState.v.z - moonState.v.z);
                m_gatewayOk = true;
            }
        }
        // Rising edge — flag for main.cpp to announce and drop to 1×.
        if (m_inMoonSoi && !m_wasInMoonSoi)
            m_soiEntryPending = true;
        m_wasInMoonSoi = m_inMoonSoi;
    } catch (...) {}

    m_burnCtrl.tick(m_currentET, m_shipR, m_shipV);

    // TCM: Earth-centric Lambert from current position to Gateway's ECI position at arrET.
    // Always uses geocentric frame — valid throughout the transfer including inside Moon SOI.
    // (Moon-centric Lambert would yield a hyperbolic LOI-style ΔV, not a course correction.)
    m_tcmValid = false;
    if (m_detail.valid
            && m_burnCtrl.phase() != BurnPhase::Armed
            && m_burnCtrl.phase() != BurnPhase::PreIgnition
            && m_burnCtrl.phase() != BurnPhase::Executing
            && m_gatewayConfig && m_gatewayConfig->hasOrbit) {
        const double tofRem = m_detail.arrET - m_currentET;
        if (tofRem > 600.0 && glm::length(m_shipR) > 100.0) {
            try {
                astro::PosState gwArr = spacecraft::vehicleStateAtEt(
                    *m_gatewayConfig, astro::EphemerisTime(m_detail.arrET));
                glm::dvec3 gwArrR(gwArr.r.x, gwArr.r.y, gwArr.r.z);

                glm::dvec3 vReq, vArr;
                if (spacecraft::solveLambert(kGmEarth, m_shipR, gwArrR,
                                              tofRem, true, vReq, vArr)) {
                    m_tcmDv    = vReq - m_shipV;
                    m_tcmValid = true;
                }
            } catch (...) {}
        }
    }

    // Arm ignition event synchronisation.
    if (m_burnCtrl.phase() == BurnPhase::Armed && m_detail.valid) {
        const double ignET = computeTliIgnitionET();
        if (ignET > m_currentET)
            m_burnCtrl.updateIgnitionET(ignET, m_eventQueue);
    }

    // While in Moon SOI, keep the "NRHO INS" OBC event updated to the live
    // ignition time (t_CA − half burn).  Only reschedule when the estimate
    // has shifted by more than 60 s, and stop when the burn is imminent
    // (within 2× burn duration) to avoid disturbing the OBC countdown.
    if (m_inMoonSoi && m_gatewayOk && m_eventQueue
            && m_mainThrustN > 0.0 && m_shipMass > 1.0) {
        const glm::dvec3 relR = m_shipMoonR - m_gatewayMoonR;
        const glm::dvec3 relV = m_shipMoonV - m_gatewayMoonV;
        const double relVMag  = glm::length(relV);
        if (relVMag > 1e-6) {
            const double t_ca = -glm::dot(relR, relV) / (relVMag * relVMag);
            if (t_ca > 0.0) {
                const double accelKmS2 = (m_mainThrustN / m_shipMass) * 1e-3;
                const double burnSec   = relVMag / accelKmS2;
                const double tIgn      = t_ca - burnSec * 0.5;
                if (tIgn > 30.0) {  // stop only in the last 30 s before ignition
                    const double ignET = m_currentET + tIgn;
                    const double curET = m_eventQueue->etForName("NRHO INS");
                    if (curET > 0.0 && std::abs(curET - ignET) > 60.0)
                        m_eventQueue->updateEventTime("NRHO INS", ignET);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// getPlan / restorePlan — quicksave support
// ---------------------------------------------------------------------------
GatewayPlanSnapshot GatewayMFD::getPlan() const
{
    GatewayPlanSnapshot snap;
    snap.valid   = m_hasData;
    snap.params  = m_params;
    snap.selDep  = m_selDep;
    snap.selTof  = m_selTof;
    snap.parkIdx = m_parkIdx;
    return snap;
}

void GatewayMFD::restorePlan(const GatewayPlanSnapshot& snap)
{
    if (!snap.valid) return;
    m_params  = snap.params;
    m_parkIdx = snap.parkIdx;

    // Re-wire the arrival override function (not serialisable — rebuild from config).
    if (m_gatewayConfig && m_gatewayConfig->hasOrbit) {
        const auto* vc = m_gatewayConfig;
        m_params.arrivalOverrideFn = [vc](double etArr) -> astro::PosState {
            return spacecraft::vehicleStateAtEt(*vc, astro::EphemerisTime(etArr));
        };
    }

    compute();  // blocking recompute, same as pressing COMP

    m_selDep = snap.selDep;
    m_selTof = snap.selTof;
    if (m_hasData && m_selDep >= 0 && m_selTof >= 0) {
        resolveSelected();
        if (m_detail.valid)
            m_page = 3;   // drop onto coast page — if loading mid-transit
    }
}

// ---------------------------------------------------------------------------
// compute — synchronous porkchop grid
// ---------------------------------------------------------------------------
void GatewayMFD::compute()
{
    if (m_computing) return;
    m_computing = true;
    m_hasData   = false;
    m_error.clear();

    m_params.centralBody          = 399;
    m_params.arrivalBody          = 301;   // fallback only; override takes priority
    m_params.departureBody        = 399;
    m_params.muCentral            = kGmEarth;
    m_params.useDepartureOverride = true;
    m_params.departureR           = m_shipR;
    m_params.departureV           = m_shipV;
    m_params.nDep                 = 90;
    m_params.nTof                 = 40;

    if (m_gatewayConfig && m_gatewayConfig->hasOrbit) {
        const auto* vc = m_gatewayConfig;
        m_params.arrivalOverrideFn = [vc](double etArr) -> astro::PosState {
            return spacecraft::vehicleStateAtEt(*vc, astro::EphemerisTime(etArr));
        };
    } else {
        m_params.arrivalOverrideFn = nullptr;
    }

    if (m_params.t1 == 0.0) {
        m_params.t0     = m_currentET;
        m_params.t1     = m_currentET + 90.0 * kDay;
        m_params.tofMin = 3.0 * kDay;
        m_params.tofMax = 8.0 * kDay;
    }

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
        computePlaneGrid();

    m_computing = false;
}

// ---------------------------------------------------------------------------
// resolveSelected
// ---------------------------------------------------------------------------
void GatewayMFD::resolveSelected()
{
    m_detail = {};
    if (!m_hasData || m_selDep < 0 || m_selTof < 0) return;
    if (!m_gatewayConfig || !m_gatewayConfig->hasOrbit) return;

    const double depET  = m_data.depET(m_selDep);
    const double tofSec = m_data.tofS(m_selTof);
    const double arrET  = depET + tofSec;

    const glm::dvec3 compR = m_params.departureR;
    const glm::dvec3 compV = m_params.departureV;

    // Gateway ECI state at arrival.
    astro::PosState gwArr;
    try {
        gwArr = spacecraft::vehicleStateAtEt(*m_gatewayConfig,
                                              astro::EphemerisTime(arrET));
    } catch (...) { return; }

    glm::dvec3 gwArrR(gwArr.r.x, gwArr.r.y, gwArr.r.z);
    glm::dvec3 gwArrV(gwArr.v.x, gwArr.v.y, gwArr.v.z);

    glm::dvec3 vDep, vArr;
    if (!spacecraft::solveLambert(kGmEarth, compR, gwArrR, tofSec, true, vDep, vArr))
        return;

    // Iterate to find the burn point on the parking orbit.
    glm::dvec3 burnR = compR;
    {
        const glm::dvec3 h    = glm::cross(compR, compV);
        const double     hMag = glm::length(h);
        if (hMag > 1e-6) {
            const glm::dvec3 hHat   = h / hMag;
            const glm::dvec3 ev     = glm::cross(compV, h) / kGmEarth - glm::normalize(compR);
            const double     ecc    = glm::length(ev);
            const glm::dvec3 periDir = (ecc > 1e-6) ? glm::normalize(ev) : glm::normalize(compR);
            const glm::dvec3 qDir    = glm::cross(hHat, periDir);
            const double     p_orb  = hMag * hMag / kGmEarth;

            for (int k = 0; k < 3; ++k) {
                glm::dvec3 proj = vDep - glm::dot(vDep, hHat) * hHat;
                if (glm::length(proj) < 1e-9) break;
                const double burnTA = std::atan2(-glm::dot(proj, periDir),
                                                  glm::dot(proj, qDir));
                const double rBurn  = p_orb / (1.0 + ecc * std::cos(burnTA));
                burnR = rBurn * (std::cos(burnTA) * periDir + std::sin(burnTA) * qDir);

                glm::dvec3 vDep_new, vArr_new;
                if (!spacecraft::solveLambert(kGmEarth, burnR, gwArrR, tofSec,
                                              true, vDep_new, vArr_new))
                    break;
                vDep = vDep_new;
                vArr = vArr_new;
            }
        }
    }

    const double vCircBurn = std::sqrt(kGmEarth / glm::length(burnR));
    const glm::dvec3 depV  = glm::normalize(vDep) * vCircBurn;
    const double rDep = glm::length(burnR);

    m_detail.valid    = true;
    m_detail.depET    = depET;
    m_detail.arrET    = arrET;
    m_detail.tofSec   = tofSec;
    m_detail.dv1      = static_cast<float>(glm::length(vDep - depV));
    m_detail.dv2      = static_cast<float>(glm::length(vArr - gwArrV));
    m_detail.c3       = static_cast<float>(glm::dot(vDep, vDep) - 2.0 * kGmEarth / rDep);
    m_detail.depPos   = burnR;
    m_detail.arrPos   = gwArrR;
    m_detail.vDep     = vDep;
    m_detail.vArr     = vArr;
    m_detail.vDepBody = depV;
    m_detail.vArrBody = gwArrV;

    // ---- Petaloid trace: sample Gateway and Moon ECI positions over
    // ±1 NRHO period (~6.5 days) centred on the arrival ET. ----
    m_gwTrace.clear();
    m_moonOrbitPts.clear();
    m_gwMoonTrace.clear();
    {
        constexpr int    kN       = 180;          // sample count
        constexpr double kSpanDay = 13.0;         // total span (days)
        const double span = kSpanDay * kDay;
        const double dt   = span / (kN - 1);
        const double t0   = arrET - span * 0.5;

        m_gwTrace.reserve(kN);
        m_moonOrbitPts.reserve(kN);
        m_gwMoonTrace.reserve(kN);

        for (int i = 0; i < kN; ++i) {
            const double t = t0 + i * dt;
            try {
                astro::PosState gw = spacecraft::vehicleStateAtEt(
                    *m_gatewayConfig, astro::EphemerisTime(t));
                const glm::dvec3 gwEci(gw.r.x, gw.r.y, gw.r.z);
                m_gwTrace.push_back(gwEci);

                astro::PosState moon;
                astro::Spice().getRelativeGeometricState(
                    301, 399, astro::EphemerisTime(t), moon, kEclipJ2000);
                const glm::dvec3 moonEci(moon.r.x, moon.r.y, moon.r.z);
                m_moonOrbitPts.push_back(moonEci);

                m_gwMoonTrace.push_back(gwEci - moonEci);
            } catch (...) {
                m_gwTrace.clear();
                m_moonOrbitPts.clear();
                m_gwMoonTrace.clear();
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// computePlaneGrid
// ---------------------------------------------------------------------------
void GatewayMFD::computePlaneGrid()
{
    m_planeGrid.clear();
    m_planeMin = 1e9f;
    m_planeMax = 0.0f;

    if (!m_hasData || !m_gatewayConfig || !m_gatewayConfig->hasOrbit) return;

    const int nDep = m_data.params.nDep;
    const int nTof = m_data.params.nTof;
    m_planeGrid.assign(nDep * nTof, spacecraft::PorkchopData::kNoSolution);

    const glm::dvec3 h = glm::cross(m_shipR, m_shipV);
    const double hMag = glm::length(h);
    if (hMag < 1e-6) return;
    const glm::dvec3 hHat   = h / hMag;
    const double     vOrbit = glm::length(m_shipV);
    if (vOrbit < 0.01) return;

    const glm::dvec3 depR = m_params.departureR;
    const glm::dvec3 depV = m_params.departureV;

    for (int iDep = 0; iDep < nDep; ++iDep) {
        const double etDep = m_data.depET(iDep);
        for (int iTof = 0; iTof < nTof; ++iTof) {
            if (m_data.get(m_data.dvTotal, iDep, iTof)
                    >= spacecraft::PorkchopData::kNoSolution * 0.5f)
                continue;

            const double tofSec = m_data.tofS(iTof);
            const double etArr  = etDep + tofSec;

            try {
                astro::PosState gwArr = spacecraft::vehicleStateAtEt(
                    *m_gatewayConfig, astro::EphemerisTime(etArr));
                glm::dvec3 gwArrR(gwArr.r.x, gwArr.r.y, gwArr.r.z);

                glm::dvec3 vDep, vArr;
                if (!spacecraft::solveLambert(kGmEarth, depR, gwArrR,
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
// computeTliIgnitionET — identical to CislunarMFD
// ---------------------------------------------------------------------------
double GatewayMFD::computeTliIgnitionET() const
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

    glm::dvec3 vDepProj    = m_detail.vDep - glm::dot(m_detail.vDep, hHat) * hHat;
    double     vDepProjMag = glm::length(vDepProj);
    if (vDepProjMag < 1e-9) return 0.0;

    double burnTA = std::atan2(-glm::dot(vDepProj, periDir), glm::dot(vDepProj, qDir));
    while (burnTA >  std::numbers::pi) burnTA -= 2.0 * std::numbers::pi;
    while (burnTA < -std::numbers::pi) burnTA += 2.0 * std::numbers::pi;

    glm::dvec3 rHat  = glm::normalize(m_shipR);
    double ta_now = std::atan2(glm::dot(rHat, qDir), glm::dot(rHat, periDir));

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

    const double dvTLI   = static_cast<double>(m_detail.dv1);
    const double accel   = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
    const double burnDur = dvTLI * 1000.0 / accel;

    return m_currentET + timeToBurn - burnDur * 0.5;
}

// ---------------------------------------------------------------------------
// Button labels
// ---------------------------------------------------------------------------
const char* GatewayMFD::leftLabel(int slot) const
{
    switch (m_page) {
    case 0:
        switch (slot) {
        case 0: return "DEP<";
        case 1: return "DEP>";
        case 2: return "TOF<";
        case 3: return "TOF>";
        case 4: return m_showPC ? "PC:ON" : "PC";
        case 5: return "RST";
        default: return "";
        }
    case 1:
        if (slot == 3) return "ALT";
        if (slot == 4) return "BACK";
        return "";
    case 2:
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
    case 3:
        if (slot == 4) return "BACK";
        return "";
    case 4:
        if (slot == 4) return "BACK";
        return "";
    default: return "";
    }
}

const char* GatewayMFD::rightLabel(int slot) const
{
    switch (m_page) {
    case 0:
        switch (slot) {
        case 0: return (m_selDep >= 0 && m_selTof >= 0 && m_hasData) ? "INFO" : "COMP";
        case 1: return "WIN<";
        case 2: return "WIN>";
        case 3: return "RNG<";
        case 4: return "RNG>";
        default: return "";
        }
    case 1:
        if (slot == 4) return "BURN";
        return "";
    case 2:
        if (slot == 4) return "CST";
        return "";
    case 3:
        if (slot == 4) return "NRHO";
        return "";
    default: return "";
    }
}

// ---------------------------------------------------------------------------
// Button actions
// ---------------------------------------------------------------------------
void GatewayMFD::onLeft(int slot)
{
    switch (m_page) {
    case 0:
        switch (slot) {
        case 0: m_params.t0 -= 15.0*kDay; m_params.t1 -= 15.0*kDay; compute(); break;
        case 1: m_params.t0 += 15.0*kDay; m_params.t1 += 15.0*kDay; compute(); break;
        case 2: m_params.tofMin = std::max(1.0*kDay, m_params.tofMin - 6.0*3600.0); compute(); break;
        case 3: m_params.tofMax = std::min(30.0*kDay, m_params.tofMax + 6.0*3600.0); compute(); break;
        case 4: m_showPC = !m_showPC; break;
        case 5:
            m_params.t0     = m_currentET;
            m_params.t1     = m_currentET + 90.0*kDay;
            m_params.tofMin = 3.0*kDay;
            m_params.tofMax = 8.0*kDay;
            m_detail = {};
            compute();
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
                    plan.name         = "TLI";
                    plan.ignitionET   = ignET;
                    plan.c3Required   = static_cast<double>(m_detail.c3);
                    plan.depBodyMu    = kGmEarth;
                    plan.dvMagnitude  = dvTLI;
                    plan.burnDuration = dvTLI * 1000.0 / accel;
                    plan.retrogradeBurn = false;
                    plan.slewOnArm      = true;
                    // Burn direction = ΔV vector (departure velocity minus parking-orbit
                    // velocity at burn point).  This correctly captures any out-of-plane
                    // component for inclined NRHO transfers; reduces to near-prograde
                    // for low-plane-change windows.
                    {
                        const glm::dvec3 dv = m_detail.vDep - m_detail.vDepBody;
                        if (glm::length(dv) > 1e-9)
                            plan.burnDirection = glm::normalize(dv);
                    }
                    if (m_eventQueue) m_eventQueue->cancelByName("TLI");
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
        if (slot == 4) m_page = 3;
        break;
    default: break;
    }
}

void GatewayMFD::onRight(int slot)
{
    switch (m_page) {
    case 0: {
        const double depMid  = (m_params.t0 + m_params.t1) * 0.5;
        const double tofMid  = (m_params.tofMin + m_params.tofMax) * 0.5;
        double       depHalf = (m_params.t1 - m_params.t0) * 0.5;
        double       tofHalf = (m_params.tofMax - m_params.tofMin) * 0.5;
        if (slot == 1) {
            depHalf = std::max(depHalf * 0.5, 1.5 * kDay);
            m_params.t0 = depMid - depHalf; m_params.t1 = depMid + depHalf;
            compute(); break;
        }
        if (slot == 2) {
            depHalf *= 2.0;
            m_params.t0 = depMid - depHalf; m_params.t1 = depMid + depHalf;
            compute(); break;
        }
        if (slot == 3) {
            tofHalf = std::max(tofHalf * 0.5, 3.0 * 3600.0);
            m_params.tofMin = tofMid - tofHalf;
            m_params.tofMax = tofMid + tofHalf;
            if (m_params.tofMin < 3600.0) {
                m_params.tofMax -= m_params.tofMin - 3600.0;
                m_params.tofMin  = 3600.0;
            }
            compute(); break;
        }
        if (slot == 4) {
            tofHalf *= 2.0;
            m_params.tofMin = tofMid - tofHalf;
            m_params.tofMax = tofMid + tofHalf;
            if (m_params.tofMin < 3600.0) {
                m_params.tofMax -= m_params.tofMin - 3600.0;
                m_params.tofMin  = 3600.0;
            }
            compute(); break;
        }
        if (slot == 0) {
            if (m_selDep >= 0 && m_selTof >= 0 && m_hasData) {
                resolveSelected();
                if (m_detail.valid) {
                    m_page = 1;
                    if (m_eventQueue) {
                        if (m_detail.depET > m_currentET)
                            m_eventQueue->schedule("TLI", m_detail.depET);
                        if (m_detail.arrET > m_currentET) {
                            // Estimate ignition = arrET − half burn, using plan dv2
                            // and current thrust/mass.  Refined live once in Moon SOI.
                            double insET = m_detail.arrET;
                            if (m_mainThrustN > 0.0 && m_shipMass > 1.0) {
                                const double accel = (m_mainThrustN / m_shipMass) * 1e-3;
                                insET -= (static_cast<double>(m_detail.dv2) / accel) * 0.5;
                            }
                            m_eventQueue->schedule("NRHO INS", insET);
                        }
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
    }
    case 1: if (slot == 4) m_page = 2; break;
    case 2: if (slot == 4) m_page = 3; break;
    case 3: if (slot == 4) m_page = 4; break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
void GatewayMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    if (m_page == 4) { renderNRHO (dl, origin, size); return; }
    if (m_page == 3) { renderCoast(dl, origin, size); return; }
    if (m_page == 2) { renderBurn (dl, origin, size); return; }
    if (m_page == 1) { renderPlan (dl, origin, size); return; }
    renderWindow(dl, origin, size);
}

// ---------------------------------------------------------------------------
// Page 0: porkchop — identical layout to CislunarMFD, title changed
// ---------------------------------------------------------------------------
void GatewayMFD::renderWindow(ImDrawList* dl, ImVec2 origin, ImVec2 size)
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
        const char* l1 = m_gatewayConfig ? "EARTH -> GATEWAY  (90 day window)"
                                         : "No Gateway config — check scenario";
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

    // Contour lines (marching squares) — identical to CislunarMFD.
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
                float v0=m_data.get(m_data.dvTotal,i,j),    v1=m_data.get(m_data.dvTotal,i+1,j);
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

    // Plane-change contours.
    if (m_showPC && !m_planeGrid.empty()) {
        const float pcLo  = m_planeMin;
        const float pcHi  = m_planeMax;
        const float pcRng = (pcHi > pcLo) ? (pcHi - pcLo) : 1.0f;
        float step = pcRng / 6.0f;
        const float steps[] = {0.01f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f};
        for (float s : steps) { if (s >= step) { step = s; break; } }
        std::vector<float> pcLevs;
        for (float lev=std::ceil(pcLo/step)*step; lev<=pcHi && pcLevs.size()<10; lev+=step)
            pcLevs.push_back(lev);
        const ImU32 pcCol = IM_COL32(80, 220, 255, 140);
        static const int8_t kMS2[16][4] = {
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
        struct LabelPt2 { float x,y; bool valid=false; };
        std::vector<LabelPt2> pcLblPts(pcLevs.size());
        const float kNS2 = spacecraft::PorkchopData::kNoSolution * 0.5f;
        for (int li=0; li<static_cast<int>(pcLevs.size()); ++li) {
            float lev=pcLevs[li];
            for (int i=0;i<nDep-1;++i) for (int j=0;j<nTof-1;++j) {
                float v0=m_planeGrid[j*nDep+i],       v1=m_planeGrid[j*nDep+i+1];
                float v2=m_planeGrid[(j+1)*nDep+i+1], v3=m_planeGrid[(j+1)*nDep+i];
                if(v0>kNS2||v1>kNS2||v2>kNS2||v3>kNS2) continue;
                int ci=((v0>lev)?1:0)|((v1>lev)?2:0)|((v2>lev)?4:0)|((v3>lev)?8:0);
                if(ci==0||ci==15) continue;
                int8_t e[4];
                if(ci==5||ci==10){
                    bool ca=((v0+v1+v2+v3)*0.25f>lev);
                    if(ci==5){if(!ca){e[0]=3;e[1]=0;e[2]=1;e[3]=2;}else{e[0]=0;e[1]=1;e[2]=2;e[3]=3;}}
                    else     {if(!ca){e[0]=0;e[1]=1;e[2]=2;e[3]=3;}else{e[0]=3;e[1]=0;e[2]=1;e[3]=2;}}
                } else { e[0]=kMS2[ci][0];e[1]=kMS2[ci][1];e[2]=kMS2[ci][2];e[3]=kMS2[ci][3]; }
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
        for (int li=0; li<static_cast<int>(pcLevs.size()); ++li) {
            if(!pcLblPts[li].valid) continue;
            char buf[10]; std::snprintf(buf,sizeof(buf),"%.2g",pcLevs[li]);
            ImVec2 tsz=ImGui::CalcTextSize(buf);
            float lx=gx0+(pcLblPts[li].x+0.5f)*cellW-tsz.x*0.5f;
            float ly=pcLblPts[li].y-tsz.y*0.5f;
            dl->AddRectFilled({lx-1,ly-1},{lx+tsz.x+1,ly+tsz.y+1},IM_COL32(0,0,30,180));
            dl->AddText({lx,ly},IM_COL32(100,230,255,230),buf);
        }
    }

    // Minimum-plane-change isoline — always drawn when plane data exists.
    // Drawn 5% of the PC range above the minimum so it sits visibly inside
    // the first regular isoline rather than coinciding with it.
    if (!m_planeGrid.empty() && m_planeMin < spacecraft::PorkchopData::kNoSolution * 0.5f) {
        const float pcRange  = (m_planeMax > m_planeMin) ? (m_planeMax - m_planeMin) : 1.0f;
        const float kZeroLev = m_planeMin + std::max(0.1f, pcRange * 0.05f);
        const ImU32 zeroCol  = IM_COL32(255, 120, 220, 220);   // magenta
        const float kNS3 = spacecraft::PorkchopData::kNoSolution * 0.5f;
        static const int8_t kMS3[16][4] = {
            {-1,-1,-1,-1},{3,0,-1,-1},{0,1,-1,-1},{3,1,-1,-1},
            {1,2,-1,-1},  {0,0,0,0}, {0,2,-1,-1},{3,2,-1,-1},
            {2,3,-1,-1},  {2,0,-1,-1},{0,0,0,0}, {2,1,-1,-1},
            {1,3,-1,-1},  {1,0,-1,-1},{0,3,-1,-1},{-1,-1,-1,-1},
        };
        auto zEdgePt = [&](int i, int j, int edge,
                           float v0, float v1, float v2, float v3) -> ImVec2 {
            float bx=gx0+(i+0.5f)*cellW, by=gy0+(nTof-0.5f-j)*cellH;
            float tx=bx+cellW, ty=by-cellH;
            auto lerp=[](float a,float b,float lev){float d=b-a; return d==0?0.5f:(lev-a)/d;};
            switch(edge){
            case 0:return{bx+lerp(v0,v1,kZeroLev)*cellW, by};
            case 1:return{tx, by-lerp(v1,v2,kZeroLev)*cellH};
            case 2:return{tx-lerp(v2,v3,kZeroLev)*cellW, ty};
            case 3:return{bx, ty+lerp(v3,v0,kZeroLev)*cellH};
            default:return{bx,by};}
        };
        ImVec2 lblPt{0,0}; bool lblValid=false;
        for (int i=0;i<nDep-1;++i) for (int j=0;j<nTof-1;++j) {
            float v0=m_planeGrid[j*nDep+i],       v1=m_planeGrid[j*nDep+i+1];
            float v2=m_planeGrid[(j+1)*nDep+i+1], v3=m_planeGrid[(j+1)*nDep+i];
            if(v0>kNS3||v1>kNS3||v2>kNS3||v3>kNS3) continue;
            int ci=((v0>kZeroLev)?1:0)|((v1>kZeroLev)?2:0)|
                   ((v2>kZeroLev)?4:0)|((v3>kZeroLev)?8:0);
            if(ci==0||ci==15) continue;
            int8_t e[4];
            if(ci==5||ci==10){
                bool ca=((v0+v1+v2+v3)*0.25f>kZeroLev);
                if(ci==5){if(!ca){e[0]=3;e[1]=0;e[2]=1;e[3]=2;}else{e[0]=0;e[1]=1;e[2]=2;e[3]=3;}}
                else     {if(!ca){e[0]=0;e[1]=1;e[2]=2;e[3]=3;}else{e[0]=3;e[1]=0;e[2]=1;e[3]=2;}}
            } else { e[0]=kMS3[ci][0];e[1]=kMS3[ci][1];e[2]=kMS3[ci][2];e[3]=kMS3[ci][3]; }
            if(e[0]>=0&&e[1]>=0){
                ImVec2 pa=zEdgePt(i,j,e[0],v0,v1,v2,v3);
                ImVec2 pb=zEdgePt(i,j,e[1],v0,v1,v2,v3);
                dl->AddLine(pa,pb,zeroCol,1.5f);
                if(!lblValid||i>(int)lblPt.x){lblPt={float(i),(pa.y+pb.y)*0.5f};lblValid=true;}
            }
            if(e[2]>=0&&e[3]>=0){
                ImVec2 pc=zEdgePt(i,j,e[2],v0,v1,v2,v3);
                ImVec2 pd=zEdgePt(i,j,e[3],v0,v1,v2,v3);
                dl->AddLine(pc,pd,zeroCol,1.5f);
            }
        }
        if(lblValid){
            char lbl[20]; std::snprintf(lbl,sizeof(lbl),"PC min %.2f",m_planeMin);
            ImVec2 tsz=ImGui::CalcTextSize(lbl);
            float lx=gx0+(lblPt.x+0.5f)*cellW-tsz.x*0.5f;
            float ly=lblPt.y-tsz.y*0.5f;
            dl->AddRectFilled({lx-1,ly-1},{lx+tsz.x+1,ly+tsz.y+1},IM_COL32(0,0,20,180));
            dl->AddText({lx,ly},zeroCol,lbl);
        }
    }

    // Hover / click / zoom-box.
    const ImVec2 mp = ImGui::GetMousePos();
    const bool inGrid = mp.x>=gx0&&mp.x<gx1&&mp.y>=gy0&&mp.y<gy1;
    int hDep=-1, hTof=-1;
    if (inGrid) {
        hDep = std::clamp(static_cast<int>((mp.x-gx0)/cellW), 0, nDep-1);
        hTof = std::clamp(nTof-1-static_cast<int>((mp.y-gy0)/cellH), 0, nTof-1);
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !m_zoomActive)
            { m_selDep=hDep; m_selTof=hTof; }
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && inGrid) {
        m_zoomStart = mp; m_zoomActive = true;
    }
    if (m_zoomActive) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            ImVec2 lo={std::min(m_zoomStart.x,mp.x),std::min(m_zoomStart.y,mp.y)};
            ImVec2 hi={std::max(m_zoomStart.x,mp.x),std::max(m_zoomStart.y,mp.y)};
            dl->AddRectFilled(lo,hi,IM_COL32(255,230,80,35));
            dl->AddRect(lo,hi,IM_COL32(255,230,80,220),0.0f,0,1.5f);
        } else {
            m_zoomActive = false;
            const float dx=std::abs(mp.x-m_zoomStart.x), dy=std::abs(mp.y-m_zoomStart.y);
            if (dx>5.0f&&dy>5.0f&&m_hasData) {
                const float x0=std::clamp(std::min(m_zoomStart.x,mp.x),gx0,gx1);
                const float x1=std::clamp(std::max(m_zoomStart.x,mp.x),gx0,gx1);
                const float y0=std::clamp(std::min(m_zoomStart.y,mp.y),gy0,gy1);
                const float y1=std::clamp(std::max(m_zoomStart.y,mp.y),gy0,gy1);
                const double tSpan=m_data.params.t1-m_data.params.t0;
                const double tfSpan=m_data.params.tofMax-m_data.params.tofMin;
                m_params.t0    =m_data.params.t0+(x0-gx0)/gw*tSpan;
                m_params.t1    =m_data.params.t0+(x1-gx0)/gw*tSpan;
                m_params.tofMax=m_data.params.tofMin+(1.0-(y0-gy0)/gh)*tfSpan;
                m_params.tofMin=m_data.params.tofMin+(1.0-(y1-gy0)/gh)*tfSpan;
                m_params.tofMin=std::max(m_params.tofMin,3600.0);
                if(m_params.tofMax<=m_params.tofMin) m_params.tofMax=m_params.tofMin+3600.0;
                m_selDep=-1; m_selTof=-1;
                compute();
            }
        }
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

    // Axis ticks.
    for (int i=0;i<=4;++i) {
        float fx=gx0+gw*i/4.0f;
        double et=m_data.params.t0+(m_data.params.t1-m_data.params.t0)*i/4.0;
        std::string ts=astro::EphemerisTime(et).toISOUTCString(0);
        char buf[8]; std::snprintf(buf,sizeof(buf),"%.7s",ts.c_str());
        dl->AddLine({fx,gy1},{fx,gy1+3},kDim,1.0f);
        dl->AddText({fx-14,gy1+2},kDim,buf);
    }
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
            std::snprintf(buf,sizeof(buf),"TLI %.3f  INS %.3f  TOT %.3f km/s",dv1,dv2,dvTot);
            dl->AddText({origin.x+pad,infoY+labelH},kYellow,buf);
            const int pidx = ti * nDep + di;
            const float kNS3 = spacecraft::PorkchopData::kNoSolution * 0.5f;
            if (!m_planeGrid.empty() && m_planeGrid[pidx] < kNS3) {
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
            char buf2[64]; std::snprintf(buf2,sizeof(buf2),"PC range %.3f-%.3f km/s  [PC]",
                                         m_planeMin,m_planeMax);
            dl->AddText({origin.x+pad,infoY+labelH},IM_COL32(100,230,255,180),buf2);
        } else {
            dl->AddText({origin.x+pad,infoY+labelH},kDim,"click to select");
        }
    }
}

// ---------------------------------------------------------------------------
// Page 1: plan summary
// ---------------------------------------------------------------------------
void GatewayMFD::renderPlan(ImDrawList* dl, ImVec2 origin, ImVec2 size)
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

    const double rPark = kREarth + kParkAlts[m_parkIdx];
    const double vCirc = std::sqrt(kGmEarth / rPark);
    const double vPeri = std::sqrt(static_cast<double>(m_detail.c3) + 2.0*kGmEarth/rPark);
    const double dvTLI = vPeri - vCirc;

    const double ignET = computeTliIgnitionET();
    char tIgnBuf[20] = "---";
    if (ignET > m_currentET) fmtTplus(ignET - m_currentET, tIgnBuf, sizeof(tIgnBuf));

    std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
    std::string arrStr = astro::EphemerisTime(m_detail.arrET).toISOUTCString(0);
    int tofH = static_cast<int>(m_detail.tofSec / 3600.0 + 0.5);

    txt(kGreen,  "TLI PLAN  EARTH -> GATEWAY");
    y += 2;
    txt(kCyan,   "DEP  %.10s", depStr.c_str());
    txt(kCyan,   "ARR  %.10s", arrStr.c_str());
    txt(kCyan,   "TOF  %dh  (%.2f days)", tofH, m_detail.tofSec / kDay);
    y += 2;
    txt(kYellow, "Park alt   %.0f km  [ALT]", kParkAlts[m_parkIdx]);
    txt(kYellow, "TLI ΔV     %.3f km/s", dvTLI);
    txt(kOrange, "NRHO INS   %.3f km/s  (rendezvous dV)", static_cast<double>(m_detail.dv2));
    y += 2;
    txt(kGreen,  "IGN        %s from now", tIgnBuf);

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
        diag.addMarker(m_detail.arrPos, IM_COL32(255,200,80,220), "G");
        diag.render(dl, {origin.x, y}, {size.x, diagH}, &m_planViewRot);
    }
}

// ---------------------------------------------------------------------------
// Page 2: TLI burn execution — identical to CislunarMFD::renderBurn
// ---------------------------------------------------------------------------
void GatewayMFD::renderBurn(ImDrawList* dl, ImVec2 origin, ImVec2 size)
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

    const double rShip    = glm::length(m_shipR);
    const double vShipMag = glm::length(m_shipV);
    if (rShip < 100.0 || vShipMag < 0.01) {
        dl->AddText({origin.x+4, origin.y+4}, kDim, "No ship state");
        return;
    }

    const double mu = kGmEarth;
    glm::dvec3 h    = glm::cross(m_shipR, m_shipV);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) { dl->AddText({origin.x+4, origin.y+4}, kDim, "Degenerate orbit"); return; }

    glm::dvec3 hHat  = h / hMag;
    double     p_orb = hMag * hMag / mu;
    glm::dvec3 ev    = glm::cross(m_shipV, h) / mu - glm::normalize(m_shipR);
    double     ecc   = glm::length(ev);
    glm::dvec3 periDir = (ecc > 1e-6) ? glm::normalize(ev) : glm::normalize(m_shipR);
    glm::dvec3 qDir    = glm::cross(hHat, periDir);
    double     sma     = (ecc < 1.0) ? p_orb / (1.0 - ecc*ecc) : p_orb;
    double     altKm   = rShip - kREarth;

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

    const double dvTLI = static_cast<double>(m_detail.dv1);
    glm::dvec3 vDepHat = glm::normalize(m_detail.vDep);

    glm::dvec3 hProj   = hHat - glm::dot(hHat, vDepHat) * vDepHat;
    double     hProjL  = glm::length(hProj);
    glm::dvec3 hTarget = (hProjL > 1e-9) ? hProj/hProjL
                       : glm::normalize(glm::cross(vDepHat, glm::dvec3(0,0,1)));

    double inc_tgt  = std::acos(std::clamp(hTarget.z, -1.0, 1.0)) * 180.0/std::numbers::pi;
    double raan_tgt = std::atan2(hTarget.x, -hTarget.y) * 180.0/std::numbers::pi;
    if (raan_tgt < 0.0) raan_tgt += 360.0;
    double planeErr = std::asin(std::clamp(std::abs(glm::dot(vDepHat, hHat)), 0.0, 1.0))
                      * 180.0 / std::numbers::pi;

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

    const double rPark = kREarth + kParkAlts[m_parkIdx];
    glm::dvec3 vDepProj  = m_detail.vDep - glm::dot(m_detail.vDep, hHat) * hHat;
    double vDepProjMag   = glm::length(vDepProj);
    double burnTA=0.0, burnTADeg=0.0;
    if (vDepProjMag > 1e-9) {
        burnTA = std::atan2(-glm::dot(vDepProj,periDir), glm::dot(vDepProj,qDir));
        while (burnTA >  std::numbers::pi) burnTA -= 2.0*std::numbers::pi;
        while (burnTA < -std::numbers::pi) burnTA += 2.0*std::numbers::pi;
        burnTADeg = burnTA*180.0/std::numbers::pi; if(burnTADeg<0.0) burnTADeg+=360.0;
    }
    double rBurn   = p_orb/(1.0+ecc*std::cos(burnTA));
    double altBurn = rBurn - kREarth;
    glm::dvec3 burnPos = rBurn*(std::cos(burnTA)*periDir + std::sin(burnTA)*qDir);

    const double vCirc = std::sqrt(mu/rPark);
    const double vPeri = std::sqrt(std::max(0.0, static_cast<double>(m_detail.c3) + 2.0*mu/rPark));

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
    const double burnDuration = dvTLI * 1000.0 / accelMs2;
    const double timeToStartBurn = timeToBurnPoint - burnDuration * 0.5;

    const double c3Now = vShipMag*vShipMag - 2.0*mu/rShip;
    const double c3Req = static_cast<double>(m_detail.c3);
    const double dvRemaining = (c3Now < c3Req)
        ? std::max(0.0, std::sqrt(c3Req + 2.0*mu/rShip) - vShipMag) : 0.0;
    const double burnRemaining = dvRemaining * 1000.0 / accelMs2;

    // Diagram.
    {
        OrbitDiagram diag;
        OrbitDiagram::Orbit o;
        o.r=m_shipR; o.v=m_shipV; o.mu=mu;
        o.colour=kGreen; o.showApses=(ecc>0.005); o.showCurrent=true;
        diag.addOrbit(o);
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
        if (vDepProjMag > 1e-9)
            diag.addArrow({0.0,0.0,0.0}, vDepHat, 22.0f, kOrange);
        OrbitDiagram::CentralBody cb;
        cb.radiusKm=kREarth; cb.rimColour=IM_COL32(60,140,255,200);
        cb.axisColour=IM_COL32(60,140,255,100); cb.drawAxes=false;
        diag.setCentralBody(cb);
        diag.render(dl, origin, size, &m_burnViewRot);
    }

    // Text overlay.
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
            {
                const bool fixedDir = glm::length(m_burnCtrl.getActiveBurnDirection()) > 0.5;
                add(kArmed,"PRE-IGN  T-%s  %s  [DSARM]",hms,
                    fixedDir ? "FIXED DIR" : "PROGRADE"); sep();
            }
            break;
        }
        case BurnPhase::Executing: add(kExec,"EXECUTING BURN  [DSARM]"); sep(); break;
        case BurnPhase::Complete:  add(kDone,"BURN COMPLETE"); sep(); break;
        default: break;
        }
    }
    {
        std::string nowStr=astro::EphemerisTime(m_currentET).toISOUTCString(0);
        std::string depStr=astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
        char tp[16]; fmtTplus(m_detail.depET-m_currentET,tp,sizeof(tp));
        add(kDim,  "NOW  %.10s",nowStr.c_str());
        add(kGreen,"DEP  %.10s  (%s)",depStr.c_str(),tp);
    }
    add(kCyan,"TLI ΔV %.3f km/s  C3 %.2f km²/s²", dvTLI, m_detail.c3);
    sep();
    add(kCyan, "PARK ORBIT");
    add(kGreen," i %.1f  Ω %.1f  ω %.1f  e %.4f",inc_cur,raan_cur,argpe_cur,ecc);
    add(kGreen," TA %.1f  Alt %.0f km  SMA %.0f km",taDeg_now,altKm,sma);
    sep();
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
    add(kCyan,"NODES");
    if (hasNodes) {
        char bAN[12],bDN[12]; fmtHMS(tToAN,bAN,sizeof(bAN)); fmtHMS(tToDN,bDN,sizeof(bDN));
        add(IM_COL32(100,255,100,230)," AN  TA %.1f  T+ %s",taDeg_AN,bAN);
        add(IM_COL32(255,100,100,230)," DN  TA %.1f  T+ %s",taDeg_DN,bDN);
    } else { add(kDim," (coplanar)"); }
    sep();
    add(kCyan,  "BURN  TA %.1f  Alt %.0f km",burnTADeg,altBurn);
    add(kDim,   "TLI  alt %.0f km  [ALT]",kParkAlts[m_parkIdx]);
    add(kYellow," dV-TLI %.3f km/s  (Vp %.3f  Vc %.3f)",dvTLI,vPeri,vCirc);
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
        ImU32 dvCol = (dvRemaining<0.001)?kGreen:(dvRemaining<dvTLI*0.1)?kYellow:kOrange;
        add(dvCol,"dV-REMAINING %.3f km/s  (%s)",dvRemaining,bRem);
        if (dvRemaining<0.001) add(kGreen," BURN COMPLETE");
    }

    float maxW=0.0f;
    for (auto& l:lines) if(l.col) maxW=std::max(maxW,ImGui::CalcTextSize(l.txt).x);
    float bx0=origin.x+pad, by0=origin.y+pad;
    float bx1=bx0+maxW+pad*2.0f, by1=by0+static_cast<float>(lines.size())*lineH+pad;
    dl->AddRectFilled({bx0,by0},{bx1,by1},kBacking,3.0f);
    float tx=bx0+pad, ty=by0+pad*0.5f;
    for (auto& l:lines) { if(l.col) dl->AddText({tx,ty},l.col,l.txt); ty+=lineH; }
}

// ---------------------------------------------------------------------------
// Page 3: coast — same as CislunarMFD but targets Gateway
// ---------------------------------------------------------------------------
void GatewayMFD::renderCoast(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen   = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim     = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow  = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan    = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange  = IM_COL32(255, 140,   0, 220);
    const ImU32 kBacking = IM_COL32(  0,   0,   0, 175);

    const float pad   = 3.0f;
    const float lineH = 11.0f;

    if (!m_detail.valid) {
        dl->AddText({origin.x+pad, origin.y+pad}, kDim, "No plan loaded");
        return;
    }

    // Current Gateway ECI state.
    glm::dvec3 gwPos(0.0), gwVel(0.0);
    bool gwOk = false;
    if (m_gatewayConfig && m_gatewayConfig->hasOrbit) {
        try {
            astro::PosState gs = spacecraft::vehicleStateAtEt(
                *m_gatewayConfig, astro::EphemerisTime(m_currentET));
            gwPos = glm::dvec3(gs.r.x, gs.r.y, gs.r.z);
            gwVel = glm::dvec3(gs.v.x, gs.v.y, gs.v.z);
            gwOk = true;
        } catch (...) {}
    }

    // Orbit diagram.
    {
        OrbitDiagram diag;
        diag.setCentralBody({kREarth, IM_COL32(80,120,220,200)});

        // Moon orbit trace — stable ellipse, drawn as an Arc from the
        // precomputed sample points so it matches the petaloid time range.
        if (!m_moonOrbitPts.empty()) {
            OrbitDiagram::Arc moonArc;
            moonArc.pts       = m_moonOrbitPts;
            moonArc.colour    = IM_COL32(120, 120, 160, 80);
            moonArc.thickness = 1.0f;
            diag.addArc(moonArc);
        }

        // Gateway petaloid — ECI positions over ±1 NRHO period around arrival.
        // Split into "past" (dimmer) and "future" (brighter) segments at current ET
        // so the pilot can see which petal they're aiming for.
        if (!m_gwTrace.empty() && m_detail.valid) {
            constexpr int    kN       = 180;
            constexpr double kSpanDay = 13.0;
            const double span = kSpanDay * kDay;
            const double t0   = m_detail.arrET - span * 0.5;
            const double dt   = span / (kN - 1);

            // Find split index: first sample at or after current ET.
            int splitIdx = kN;
            for (int i = 0; i < (int)m_gwTrace.size(); ++i) {
                if (t0 + i * dt >= m_currentET) { splitIdx = i; break; }
            }

            // Past segment (dim gold).
            if (splitIdx > 1) {
                OrbitDiagram::Arc past;
                past.pts.assign(m_gwTrace.begin(),
                                m_gwTrace.begin() + splitIdx + 1);
                past.colour    = IM_COL32(180, 140, 40, 60);
                past.thickness = 1.0f;
                diag.addArc(past);
            }
            // Future segment (bright gold).
            if (splitIdx < (int)m_gwTrace.size() - 1) {
                OrbitDiagram::Arc future;
                future.pts.assign(m_gwTrace.begin() + splitIdx,
                                  m_gwTrace.end());
                future.colour    = IM_COL32(255, 200, 60, 180);
                future.thickness = 1.5f;
                diag.addArc(future);
            }
        }

        // Ship transfer arc (current osculating orbit, ship dot).
        if (glm::length(m_shipR) > 100.0)
            diag.addOrbit(m_shipR, m_shipV, kGmEarth,
                         IM_COL32(0, 200, 80, 200), "", false, true);

        // Moon current position — linearly interpolated between samples.
        if (!m_moonOrbitPts.empty() && m_detail.valid) {
            constexpr int    kN       = 180;
            constexpr double kSpanDay = 13.0;
            const double span = kSpanDay * kDay;
            const double t0   = m_detail.arrET - span * 0.5;
            const double dt   = span / (kN - 1);
            const double frac = (m_currentET - t0) / dt;
            const int    i0   = std::clamp(static_cast<int>(frac), 0, (int)m_moonOrbitPts.size() - 2);
            const int    i1   = i0 + 1;
            const double t    = std::clamp(frac - i0, 0.0, 1.0);
            const glm::dvec3 moonInterp = glm::mix(m_moonOrbitPts[i0], m_moonOrbitPts[i1], t);
            diag.addMarker(moonInterp, IM_COL32(180,180,220,200), "M");
        }

        // Gateway current position "G" and planned arrival "T".
        if (gwOk)
            diag.addMarker(gwPos, IM_COL32(255, 200, 80, 240), "G");
        if (m_detail.valid)
            diag.addMarker(m_detail.arrPos, IM_COL32(100, 200, 255, 220), "T");

        // TCM correction arrow.
        if (m_tcmValid && glm::length(m_tcmDv) > 1e-4)
            diag.addArrow(m_shipR, glm::normalize(m_tcmDv), 20.0f,
                         IM_COL32(255, 160, 0, 220));

        diag.render(dl, origin, size, &m_coastViewRot);
    }

    struct Line { ImU32 col; char txt[80]; };
    std::vector<Line> lines;
    auto add = [&](ImU32 col, const char* fmt, ...) {
        Line l; l.col = col;
        va_list ap; va_start(ap,fmt);
        std::vsnprintf(l.txt,sizeof(l.txt),fmt,ap); va_end(ap);
        lines.push_back(l);
    };
    auto sep = [&]() { add(0,""); };

    add(kGreen,"COAST  EARTH -> GATEWAY");
    sep();

    {
        double progress = (m_detail.tofSec > 0.0)
            ? (m_currentET - m_detail.depET) / m_detail.tofSec : -1.0;
        double tofRem = m_detail.arrET - m_currentET;
        char tp[20];
        if (tofRem >= 0.0) {
            int days = static_cast<int>(tofRem / kDay);
            int rem  = static_cast<int>(tofRem) % static_cast<int>(kDay);
            int hh = rem/3600; rem %= 3600; int mm = rem/60; int ss = rem%60;
            if (days > 0) std::snprintf(tp,sizeof(tp),"%dd %02d:%02d",days,hh,mm);
            else          std::snprintf(tp,sizeof(tp),"%02d:%02d:%02d",hh,mm,ss);
        } else {
            std::snprintf(tp,sizeof(tp),"PAST");
        }
        if (progress >= 0.0 && progress <= 1.5)
            add(kCyan,"Progress  %.1f%%  T-arr %s",progress*100.0,tp);
        else
            add(kCyan,"T-arr %s",tp);
    }

    if (gwOk) {
        glm::dvec3 rel = m_shipR - gwPos;
        double distKm  = glm::length(rel);
        glm::dvec3 relV = m_shipV - gwVel;
        double closure = (distKm > 1.0) ? -glm::dot(rel, relV) / distKm : 0.0;
        add(kGreen,"Dist Gateway  %.0f km",distKm);
        ImU32 clCol = (closure > 0.0) ? kGreen : kOrange;
        add(clCol,"Closure       %+.3f km/s  (%s)",
            closure, closure > 0.0 ? "approaching" : "receding");
    }
    sep();

    add(kCyan, m_inMoonSoi ? "IN MOON SOI — proceed to NRHO" : "COURSE ERROR (TCM)");
    if (m_tcmValid) {
        double dvMag = glm::length(m_tcmDv) * 1000.0;
        ImU32 dvCol = (dvMag < 5.0) ? kGreen : (dvMag < 50.0) ? kYellow : kOrange;
        add(dvCol," dV  %.1f m/s",dvMag);
        if (!m_inMoonSoi && dvMag > 0.5) {
            double vMag = glm::length(m_shipV);
            double radial = (vMag > 1e-6) ? glm::dot(m_tcmDv, m_shipV) / vMag * 1000.0 : 0.0;
            double lateral = std::sqrt(std::max(0.0, dvMag*dvMag - radial*radial));
            add(dvCol," radial %+.1f  lateral %.1f m/s",radial,lateral);
        }
    } else {
        if (m_inMoonSoi)
            add(kGreen," In Moon SOI — arm NRHO insertion");
        else
            add(kGreen," On target");
    }
    sep();

    if (m_inMoonSoi)
        add(kOrange,"IN MOON SOI — proceed to NRHO page");

    float maxW = 0.0f;
    for (auto& l : lines) if (l.col) maxW = std::max(maxW, ImGui::CalcTextSize(l.txt).x);
    float bx0 = origin.x + pad, by0 = origin.y + pad;
    float bx1 = bx0 + maxW + pad*2.0f;
    float by1 = by0 + static_cast<float>(lines.size()) * lineH + pad;
    dl->AddRectFilled({bx0,by0},{bx1,by1},kBacking,3.0f);
    float tx = bx0+pad, ty = by0+pad*0.5f;
    for (auto& l : lines) { if (l.col) dl->AddText({tx,ty},l.col,l.txt); ty += lineH; }

    {
        double progress = (m_detail.tofSec > 0.0)
            ? (m_currentET - m_detail.depET) / m_detail.tofSec : -1.0;
        if (progress >= 0.0) {
            float barY = by1 + 4.0f;
            float barW = size.x - 2*pad;
            float fill = std::clamp(static_cast<float>(progress),0.0f,1.0f)*barW;
            dl->AddRectFilled({bx0,barY},{bx0+barW,barY+6},IM_COL32(40,40,40,200));
            dl->AddRectFilled({bx0,barY},{bx0+fill, barY+6},IM_COL32(0,200,80,200));
            dl->AddRect      ({bx0,barY},{bx0+barW,barY+6},IM_COL32(80,80,80,160));
        }
    }
}

// ---------------------------------------------------------------------------
// Page 4: NRHO approach — Gateway rendezvous display
// ---------------------------------------------------------------------------
void GatewayMFD::renderNRHO(ImDrawList* dl, ImVec2 origin, ImVec2 size)
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

    txt(kGreen,"NRHO  GATEWAY APPROACH");
    y += 4;

    if (!m_inMoonSoi) {
        txt(kDim,"Outside Moon SOI");
        if (m_detail.valid) {
            char tp[16]; fmtTplus(m_detail.arrET - m_currentET, tp, sizeof(tp));
            txt(kCyan,"Planned arrival  %s", tp);
            txt(kYellow,"NRHO INS ΔV  %.3f km/s  (from plan)", static_cast<double>(m_detail.dv2));
        }
    } else {
        // Ship Moon-centric.
        double rNow = glm::length(m_shipMoonR);
        double vNow = glm::length(m_shipMoonV);
        txt(kCyan,"Ship  r=%.0f km  v=%.3f km/s (Moon-ctr)",rNow,vNow);

        if (m_gatewayOk) {
            double gwR = glm::length(m_gatewayMoonR);
            double gwV = glm::length(m_gatewayMoonV);
            txt(kCyan,"GW    r=%.0f km  v=%.3f km/s (Moon-ctr)",gwR,gwV);
            y += 2;

            glm::dvec3 relR = m_shipMoonR - m_gatewayMoonR;
            glm::dvec3 relV = m_shipMoonV - m_gatewayMoonV;
            double dist    = glm::length(relR);
            double relVMag = glm::length(relV);
            double closure = (dist > 0.001)
                ? -glm::dot(relR, relV) / dist   // positive = approaching
                : 0.0;

            ImU32 distCol = (dist < 100.0) ? kGreen : (dist < 1000.0) ? kYellow : kOrange;
            txt(distCol,"Range  %.1f km",dist);
            ImU32 clCol = (closure > 0.0) ? kGreen : kOrange;
            txt(clCol,"Closure  %+.3f km/s  (%s)",
                closure, closure > 0.0 ? "approaching" : "receding");
            txt(kYellow,"Rel-V  %.3f km/s  (insertion ΔV required)",relVMag);

            // Closest approach + burn-to-null + insertion ignition time.
            // t_ca = -dot(relR, relV) / |relV|²  (negative = already past CA)
            double t_ca    = 0.0;
            double burnSec = 0.0;
            bool   hasCa   = false;
            bool   hasBurn = false;

            if (relVMag > 1e-6) {
                t_ca  = -glm::dot(relR, relV) / (relVMag * relVMag);
                hasCa = (t_ca > 0.0);
                if (hasCa) {
                    const double caDist = glm::length(relR + relV * t_ca);
                    const int caHr  = static_cast<int>(t_ca / 3600.0);
                    const int caMin = static_cast<int>(t_ca / 60.0) % 60;
                    const int caSec = static_cast<int>(t_ca) % 60;
                    ImU32 caCol = (caDist < 50.0) ? kGreen : (caDist < 500.0) ? kYellow : kOrange;
                    txt(caCol,"CA  %.1f km  in %s",caDist,
                        [&]{ static char b[16];
                             if(caHr>0) std::snprintf(b,sizeof(b),"%dh%02dm%02ds",caHr,caMin,caSec);
                             else       std::snprintf(b,sizeof(b),"%d:%02d",caMin,caSec);
                             return b; }());
                } else {
                    txt(kDim,"CA  past (diverging)");
                }
            }

            if (relVMag > 1e-6 && m_mainThrustN > 0.0 && m_shipMass > 0.0) {
                const double accelKmS2 = (m_mainThrustN / m_shipMass) * 1e-3;
                burnSec  = relVMag / accelKmS2;
                hasBurn  = true;
                const int bHr  = static_cast<int>(burnSec / 3600.0);
                const int bMin = static_cast<int>(burnSec / 60.0) % 60;
                const int bSec = static_cast<int>(burnSec) % 60;
                if (bHr > 0)
                    txt(kCyan,"Burn to null  %dh%02dm%02ds  (@ %.2f m/s²)",
                        bHr, bMin, bSec, m_mainThrustN / m_shipMass);
                else
                    txt(kCyan,"Burn to null  %d:%02d  (@ %.2f m/s²)",
                        bMin, bSec, m_mainThrustN / m_shipMass);
            }

            // NRHO INS ignition = CA - half burn (centre-of-burn at CA).
            if (hasCa && hasBurn) {
                const double tIgn = t_ca - burnSec * 0.5;
                if (tIgn > 0.0) {
                    const int iHr  = static_cast<int>(tIgn / 3600.0);
                    const int iMin = static_cast<int>(tIgn / 60.0) % 60;
                    const int iSec = static_cast<int>(tIgn) % 60;
                    ImU32 ignCol = (tIgn < 120.0) ? kOrange : kCyan;
                    if (iHr > 0)
                        txt(ignCol,"INS IGN  %dh%02dm%02ds",iHr,iMin,iSec);
                    else
                        txt(ignCol,"INS IGN  %d:%02d",iMin,iSec);
                } else {
                    txt(kOrange,"INS IGN  NOW (or past)");
                }
            }
            y += 2;

            if (m_detail.valid) {
                txt(kCyan,"Plan NRHO INS  %.3f km/s",static_cast<double>(m_detail.dv2));
            }
        } else {
            txt(kDim,"Gateway state unavailable");
        }
    }

    // Moon-centric orbit diagram.
    const float diagH = origin.y + size.y - y - 4;
    if (diagH > 40) {
        OrbitDiagram diag;
        diag.setCentralBody({kRMoon, IM_COL32(180,180,180,200)});

        // Ship: osculating hyperbolic arc (Keplerian is fine for short approach arc).
        if (m_inMoonSoi && glm::length(m_shipMoonR) > 1.0)
            diag.addOrbit(m_shipMoonR, m_shipMoonV, kGmMoon,
                         IM_COL32(0,210,75,200), "SHIP", false, true);

        // Gateway: sampled NRHO trace (Moon-centric) — Keplerian orbit is misleading
        // for the NRHO because it places the label at the osculating periapsis,
        // not at Gateway's actual position.
        if (!m_gwMoonTrace.empty()) {
            OrbitDiagram::Arc gwArc;
            gwArc.pts       = m_gwMoonTrace;
            gwArc.colour    = IM_COL32(255, 200, 80, 100);
            gwArc.thickness = 1.0f;
            diag.addArc(gwArc);
        }
        // Marker at Gateway's actual current Moon-centric position.
        if (m_inMoonSoi && m_gatewayOk)
            diag.addMarker(m_gatewayMoonR, IM_COL32(255, 200, 80, 240), "GW");

        diag.render(dl, {origin.x, y}, {size.x, diagH}, &m_nrhoViewRot);
    }
}
