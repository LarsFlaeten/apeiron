#include "TransferMFD.h"
#include "OrbitDiagram.h"
#include "BurnController.h"
#include "VoiceAnnouncer.h"

#include "apeiron/spacecraft/Autopilot.h"
#include "apeiron/spacecraft/FiniteBurnPredictor.h"

#include <astro/SpiceCore.h>
#include <astro/ReferenceFrame.h>

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <iostream>

// All heliocentric SPICE queries use ECLIPJ2000 to match the spacecraft
// integrator frame (vehicleStateAtEt rotates J2000 equatorial → ECLIPJ2000).
static const astro::ReferenceFrame kEclipJ2000 =
    astro::ReferenceFrame::createEclipJ2000();

static constexpr double kDay = 86400.0;

// ---------------------------------------------------------------------------
// Planet table for the body-select page.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Inclination preset names (parallel to TransferMFD::kIncPresetDeg[]).
// ---------------------------------------------------------------------------
static const char* const kIncPresetName[] = {
    "FREE", "PRO 0", "30", "60", "POL 90", "120", "150", "RET 180"
};
static constexpr int kNIncPresets =
    static_cast<int>(std::size(kIncPresetName));

// ---------------------------------------------------------------------------
// Format a signed time delta (seconds) as "T+1d", "T-3d", or "T+HH:MM" when
// inside the final 24 hours.
static void fmtTplus(double deltaSec, char* buf, int sz)
{
    const char sign = (deltaSec >= 0.0) ? '+' : '-';
    double abs_s = std::abs(deltaSec);
    if (abs_s >= kDay) {
        int days = static_cast<int>(abs_s / kDay);   // floor
        std::snprintf(buf, sz, "T%c%dd", sign, days);
    } else {
        int total_m = static_cast<int>(abs_s / 60.0);
        int hh = total_m / 60;
        int mm = total_m % 60;
        std::snprintf(buf, sz, "T%c%02d:%02d", sign, hh, mm);
    }
}

// ---------------------------------------------------------------------------
// Colour map: t ∈ [0,1]  →  green → yellow → red
// ---------------------------------------------------------------------------
ImU32 TransferMFD::dvColor(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    uint8_t r, g, b;
    if (t < 0.5f) {
        float u = t * 2.0f;               // 0→1 over first half
        r = static_cast<uint8_t>(255 * u);
        g = 210;
        b = 0;
    } else {
        float u = (t - 0.5f) * 2.0f;      // 0→1 over second half
        r = 255;
        g = static_cast<uint8_t>(210 * (1.0f - u));
        b = 0;
    }
    return IM_COL32(r, g, b, 230);
}

// ---------------------------------------------------------------------------
int TransferMFD::findPlanetIdx(int naifId) const
{
    for (int i = 0; i < static_cast<int>(m_bodies.size()); ++i)
        if (m_bodies[i].naifId == naifId) return i;
    return -1;
}

void TransferMFD::queryDepBodyConstants()
{
    const int depId    = m_params.departureBody;
    const int planetId = (depId < 10) ? depId * 100 + 99 : depId;

    m_depBodyRadius = 6378.0;
    try {
        astro::Vec3 radii;
        astro::Spice().getPlanetaryConstants(planetId, "RADII", radii);
        m_depBodyRadius = radii.x;
    } catch (...) {
        try {
            astro::Vec3 radii;
            astro::Spice().getPlanetaryConstants(depId, "RADII", radii);
            m_depBodyRadius = radii.x;
        } catch (...) {}
    }

    m_muDep = 398600.4418;
    try { astro::Spice().getPlanetaryConstants(planetId, "GM", m_muDep); }
    catch (...) {
        try { astro::Spice().getPlanetaryConstants(depId, "GM", m_muDep); }
        catch (...) {}
    }

    // SOI = r_body * (mu_dep / mu_central)^0.4
    m_depSOI = 929000.0;
    try {
        double muCentral = m_params.muCentral;
        if (muCentral <= 0.0)
            astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", muCentral);
        if (muCentral > 0.0 && m_muDep > 0.0) {
            astro::PosState depState;
            astro::Spice().getRelativeGeometricState(
                depId, m_params.centralBody,
                astro::EphemerisTime(m_currentET), depState, kEclipJ2000);
            double rDep = glm::length(glm::dvec3(depState.r.x, depState.r.y, depState.r.z));
            if (rDep > 1e6)
                m_depSOI = rDep * std::pow(m_muDep / muCentral, 0.4);
        }
    } catch (...) {}
}

void TransferMFD::updateDefaultTof()
{
    // Approximate Hohmann transfer TOF: half the period of the transfer ellipse.
    // Use current heliocentric positions as proxies for SMA.
    try {
        double muCentral = m_params.muCentral;
        if (muCentral <= 0.0)
            astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", muCentral);
        if (muCentral <= 0.0) return;

        astro::PosState depState, arrState;
        const astro::EphemerisTime et(m_currentET > 0.0 ? m_currentET : 0.0);
        astro::Spice().getRelativeGeometricState(
            m_params.departureBody, m_params.centralBody, et, depState, kEclipJ2000);
        astro::Spice().getRelativeGeometricState(
            m_params.arrivalBody, m_params.centralBody, et, arrState, kEclipJ2000);

        double rDep = glm::length(glm::dvec3(depState.r.x, depState.r.y, depState.r.z));
        double rArr = glm::length(glm::dvec3(arrState.r.x, arrState.r.y, arrState.r.z));
        double aXfer = (rDep + rArr) * 0.5;
        double tHohmann = M_PI * std::sqrt(aXfer * aXfer * aXfer / muCentral);  // seconds

        // Window: 0.5× to 2× Hohmann time, minimum 30 days.
        m_params.tofMin = std::max(tHohmann * 0.5, 30.0 * kDay);
        m_params.tofMax = std::max(tHohmann * 2.0, m_params.tofMin + 60.0 * kDay);
    } catch (...) {}
}

void TransferMFD::selectDeparture(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(m_bodies.size())) return;
    m_depPlanetIdx         = idx;
    m_params.departureBody = m_bodies[idx].naifId;
    m_params.muCentral     = 0.0;
    m_depBodyName          = m_bodies[idx].shortName;
    queryDepBodyConstants();
    updateDefaultTof();
    m_hasData = false;
    m_detail  = {};
}

void TransferMFD::selectArrival(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(m_bodies.size())) return;
    m_arrPlanetIdx        = idx;
    m_params.arrivalBody  = m_bodies[idx].naifId;
    m_arrBodyName         = m_bodies[idx].shortName;
    updateDefaultTof();
    m_hasData = false;
    m_detail  = {};
}

// ---------------------------------------------------------------------------
// computeIgnitionET — next burn-TA passage time for the current plan / orbit.
//
// Mirrors the Kepler time-to-burn calculation in renderDeparture so the ARM
// action and the display always agree.  Returns 0.0 on failure.
// ---------------------------------------------------------------------------
double TransferMFD::computeIgnitionET() const
{
    if (!m_detail.valid) return 0.0;

    const double mu    = m_muDep;
    const double rPark = m_depBodyRadius + kParkAlts[m_parkIdx];

    glm::dvec3 h    = glm::cross(m_shipR, m_shipV);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) return 0.0;

    glm::dvec3 hHat    = h / hMag;
    double     p_orb   = hMag * hMag / mu;
    glm::dvec3 ev      = glm::cross(m_shipV, h) / mu - glm::normalize(m_shipR);
    double     ecc     = glm::length(ev);
    if (ecc >= 1.0) return 0.0;  // not in a closed parking orbit
    glm::dvec3 periDir = (ecc > 1e-6) ? glm::normalize(ev) : glm::normalize(m_shipR);
    glm::dvec3 qDir    = glm::cross(hHat, periDir);
    double     sma     = p_orb / (1.0 - ecc * ecc);
    if (sma <= 0.0) return 0.0;

    // V∞ and burn TA
    glm::dvec3 vInf    = m_detail.vDep - m_detail.vDepBody;
    double     vInfMag = glm::length(vInf);
    if (vInfMag < 1e-6) return 0.0;

    const double e_hyp  = 1.0 + static_cast<double>(m_detail.c3) * rPark / mu;
    const double nu_inf = std::acos(-1.0 / e_hyp);
    glm::dvec3 vInfProj = vInf - glm::dot(vInf, hHat) * hHat;
    if (glm::length(vInfProj) < 1e-9) return 0.0;

    double phi_target = std::atan2(glm::dot(vInfProj, qDir),
                                   glm::dot(vInfProj, periDir));
    double burnTA = phi_target - nu_inf;
    while (burnTA >  M_PI) burnTA -= 2.0 * M_PI;
    while (burnTA < -M_PI) burnTA += 2.0 * M_PI;

    // Current TA
    glm::dvec3 rHat  = glm::normalize(m_shipR);
    double ta_now = std::atan2(glm::dot(rHat, qDir), glm::dot(rHat, periDir));

    // Kepler: time from ta_now to burnTA
    auto meanAnom = [&](double ta) -> double {
        double tanH = std::tan(ta * 0.5) * std::sqrt((1.0 - ecc) / (1.0 + ecc));
        double E    = 2.0 * std::atan(tanH);
        return E - ecc * std::sin(E);
    };
    double orbPeriod = 2.0 * M_PI * std::sqrt(sma * sma * sma / mu);
    double dM = meanAnom(burnTA) - meanAnom(ta_now);
    if (dM <= 0.0) dM += 2.0 * M_PI;
    double timeToBurnPoint = dM / (2.0 * M_PI) * orbPeriod;

    // Burn duration (centred on burn TA — ignition = midpoint - half duration)
    const double accelMs2    = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
    const double vCirc       = std::sqrt(mu / rPark);
    const double vPeri       = std::sqrt(static_cast<double>(m_detail.c3) + 2.0 * mu / rPark);
    const double burnDuration = (vPeri - vCirc) * 1000.0 / accelMs2;

    return m_currentET + timeToBurnPoint - burnDuration * 0.5;
}

// ---------------------------------------------------------------------------
TransferPlanSnapshot TransferMFD::getPlan() const
{
    TransferPlanSnapshot snap;
    snap.valid   = m_hasData;
    snap.params  = m_params;
    snap.selDep  = m_selDep;
    snap.selTof  = m_selTof;
    snap.parkIdx = m_parkIdx;
    return snap;
}

void TransferMFD::restorePlan(const TransferPlanSnapshot& snap)
{
    if (!snap.valid) return;
    m_params  = snap.params;
    m_parkIdx = snap.parkIdx;

    // Restore planet indices from NAIF IDs.
    int di = findPlanetIdx(m_params.departureBody);
    int ai = findPlanetIdx(m_params.arrivalBody);
    if (di >= 0) { m_depPlanetIdx = di; m_depBodyName = m_bodies[di].shortName; }
    if (ai >= 0) { m_arrPlanetIdx = ai; m_arrBodyName = m_bodies[ai].shortName; }
    queryDepBodyConstants();

    const int selDep = snap.selDep;
    const int selTof = snap.selTof;
    compute();
    m_selDep = selDep;
    m_selTof = selTof;
    if (m_hasData && m_selDep >= 0 && m_selTof >= 0)
        resolveSelected();
}

// ---------------------------------------------------------------------------
void TransferMFD::setEpoch(const astro::EphemerisTime& et)
{
    m_currentET = et.getETValue();

    // Departure/arrival bodies keep their current values (defaulting to 399/499
    // from member initializers); resolved into m_bodies indices in update() once
    // the body list arrives from MFDContext.
    if (m_params.departureBody == 0) m_params.departureBody = 399;
    if (m_params.arrivalBody   == 0) m_params.arrivalBody   = 499;
    m_params.centralBody   = 10;   // Sun
    m_params.muCentral     = 0.0;
    queryDepBodyConstants();

    m_params.t0 = m_currentET;
    m_params.t1 = m_currentET + 2.0 * 365.25 * kDay;

    updateDefaultTof();

    m_params.nDep = 80;
    m_params.nTof = 60;
}

// ---------------------------------------------------------------------------
void TransferMFD::compute()
{
    if (m_computing) return;
    m_computing = true;
    m_hasData   = false;
    m_error.clear();

    try {
        m_data    = spacecraft::computePorkchop(m_params);
        m_hasData = true;
        m_selDep  = -1;
        m_selTof  = -1;
    } catch (const std::exception& e) {
        const astro::SpiceException se = dynamic_cast<const astro::SpiceException&>(e);
        m_error = e.what();
        std::cout << "[TransferMFD] Compute error: " << e.what() << "\n";
        std::cout << "[TransferMFD]    Spice short message exp: " << se.getShortMessageExplanation() << "\n";
        std::cout << "[TransferMFD]    Spice long message: " << se.getLongMessage() << "\n";
        
    } catch (...) {
        m_error = "unknown exception";
    }

    m_computing = false;
}

// ---------------------------------------------------------------------------
const char* TransferMFD::leftLabel(int slot) const
{
    if (m_page == 5) {
        switch (slot) {
        case 0: return "INC<";
        case 1: return "INC>";
        case 4: return "BACK";
        default: return "";
        }
    }
    if (m_page == 4) {
        switch (slot) {
        case 0: return "DEP<";
        case 1: return "DEP>";
        case 2: return "ARR<";
        case 3: return "ARR>";
        case 4: return "OK";
        default: return "";
        }
    }
    if (m_page == 3) {
        if (slot == 0) {
            switch (m_mccWarnIdx) {
            case 0:  return "W:OFF";
            case 1:  return "W:100";
            case 2:  return "W:200";
            case 3:  return "W:500";
            default: return "";
            }
        }
        if (slot == 4) return "BACK";
        return "";
    }
    if (m_page == 2) {
        if (slot == 4) return "BACK";
        if (slot == 3) return "ALT";
        if (slot == 2) {
            switch (m_burnCtrl.phase()) {
            case BurnPhase::Armed:
            case BurnPhase::PreIgnition:
            case BurnPhase::Executing: return "DSARM";
            case BurnPhase::Complete:  return "";
            default: return (m_detail.valid && glm::length(m_shipR) > 100.0) ? "ARM" : "";
            }
        }
        return "";
    }
    if (m_page == 1) {
        if (slot == 4) return "BACK";
        if (slot == 3) return "ALT";
        return "";
    }
    switch (slot) {
    case 0: return "DEP<";
    case 1: return "DEP>";
    case 2: return "TOF<";
    case 3: return "TOF>";
    case 4: return "BDY";
    case 5: return "RST";
    default: return "";
    }
}

const char* TransferMFD::rightLabel(int slot) const
{
    if (m_page == 5) {
        if (slot == 3) return "PE";
        if (slot == 4) {
            switch (m_moiBurnCtrl.phase()) {
            case BurnPhase::Armed:
            case BurnPhase::PreIgnition:
            case BurnPhase::Executing: return "DSARM";
            default:
                return (!m_capturedAtArrival && m_bplaneValid
                        && m_moiIgnET > m_currentET && m_moiDvCirc > 0.0
                        && inArrivalSoi())
                    ? "ARM" : "";
            }
        }
        return "";
    }
    if (m_page == 4) return "";
    if (m_page == 3) {
        if (slot == 4) return "ARR";
        return "";
    }
    if (m_page == 2) {
        if (slot == 4) return "CST";
        return "";
    }
    if (m_page == 1) {
        if (slot == 4) return "BURN";
        return "";
    }
    switch (slot) {
    case 0: return (m_selDep >= 0 && m_selTof >= 0 && m_hasData) ? "INFO" : "COMP";
    case 1: return "WIN<";
    case 2: return "WIN>";
    case 3: return "RNG<";
    case 4: return "RNG>";
    default: return "";
    }
}

void TransferMFD::onLeft(int slot)
{
    if (m_page == 5) {
        if (slot == 0) {
            m_incPresetIdx = (m_incPresetIdx + kNIncPresets - 1) % kNIncPresets;
            return;
        }
        if (slot == 1) {
            m_incPresetIdx = (m_incPresetIdx + 1) % kNIncPresets;
            return;
        }
        if (slot == 4) { m_page = 3; return; }
        return;
    }
    if (m_page == 4) {
        const int nP = static_cast<int>(m_bodies.size());
        if (nP == 0) return;
        const int di = (m_depPlanetIdx < 0) ? 0 : m_depPlanetIdx;
        const int ai = (m_arrPlanetIdx < 0) ? 0 : m_arrPlanetIdx;
        switch (slot) {
        case 0: selectDeparture((di + nP - 1) % nP); break;
        case 1: selectDeparture((di + 1)      % nP); break;
        case 2: selectArrival  ((ai + nP - 1) % nP); break;
        case 3: selectArrival  ((ai + 1)      % nP); break;
        case 4: m_page = 0; break;
        default: break;
        }
        return;
    }
    if (m_page == 3) {
        if (slot == 0) {
            m_mccWarnIdx    = (m_mccWarnIdx + 1)
                              % static_cast<int>(std::size(kMccWarnKms));
            m_mccWarnActive = false;  // reset active state on threshold change
            return;
        }
        if (slot == 4) { m_page = 2; return; }
        return;
    }
    if (m_page == 2) {
        if (slot == 4) { m_page = 1; return; }
        if (slot == 3) {
            m_parkIdx = (m_parkIdx + 1)
                        % static_cast<int>(std::size(kParkAlts));
            return;
        }
        if (slot == 2) {
            const auto ph = m_burnCtrl.phase();
            if (ph == BurnPhase::Armed || ph == BurnPhase::Executing) {
                m_burnCtrl.disarm();
            } else if (m_detail.valid && glm::length(m_shipR) > 100.0) {
                const double ignET = computeIgnitionET();
                if (ignET > m_currentET) {
                    const double mu    = m_muDep;
                    const double rPark = m_depBodyRadius + kParkAlts[m_parkIdx];
                    const double accel = (m_shipMass > 1.0)
                                        ? m_mainThrustN / m_shipMass : 0.97;
                    const double vCirc = std::sqrt(mu / rPark);
                    const double vPeri = std::sqrt(
                        static_cast<double>(m_detail.c3) + 2.0 * mu / rPark);
                    BurnPlan plan;
                    plan.name        = "TMI";
                    plan.ignitionET  = ignET;
                    plan.c3Required  = m_detail.c3;
                    plan.depBodyMu   = m_muDep;
                    plan.dvMagnitude = vPeri - vCirc;
                    plan.burnDuration= plan.dvMagnitude * 1000.0 / accel;
                    m_burnCtrl.arm(plan, m_autopilot, m_eventQueue);
                }
            }
            return;
        }
        return;
    }
    if (m_page == 1) {
        if (slot == 4) m_page = 0;
        if (slot == 3) m_parkIdx = (m_parkIdx + 1)
                                   % static_cast<int>(std::size(kParkAlts));
        return;
    }
    const double shift = 30.0 * kDay;
    switch (slot) {
    case 0:
        m_params.t0 -= shift;
        m_params.t1 -= shift;
        break;
    case 1:
        m_params.t0 += shift;
        m_params.t1 += shift;
        break;
    case 2:
        m_params.tofMin -= shift;
        m_params.tofMax -= shift;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    case 3:
        m_params.tofMin += shift;
        m_params.tofMax += shift;
        break;
    case 4:
        m_page = 4;   // BDY — open body-select page
        break;
    case 5: {         // RST — reset window to now, sensible TOF, recompute
        setEpoch(astro::EphemerisTime(m_currentET));
        m_selDep    = -1;
        m_selTof    = -1;
        m_detail    = {};
        m_hasData   = false;
        m_error.clear();
        compute();
        break;
    }
    default: break;
    }
}

void TransferMFD::onRight(int slot)
{
    if (m_page == 5) {
        if (slot == 3) {
            m_peAltIdx = (m_peAltIdx + 1) % static_cast<int>(std::size(kPeTargetAlts));
            return;
        }
        if (slot == 4) {
            const auto ph = m_moiBurnCtrl.phase();
            if (ph == BurnPhase::Armed || ph == BurnPhase::PreIgnition
                || ph == BurnPhase::Executing) {
                m_moiBurnCtrl.disarm();
            } else if (!m_capturedAtArrival && m_bplaneValid
                       && m_moiIgnET > m_currentET && m_moiDvCirc > 0.0
                       && inArrivalSoi()) {
                BurnPlan plan;
                plan.name           = "MOI";
                plan.ignitionET     = m_moiIgnET;
                const double rPeTgt = m_arrBodyRadius + kPeTargetAlts[m_peAltIdx];
                plan.c3Required     = -m_muArr / rPeTgt;
                plan.depBodyMu      = m_muArr;
                plan.dvMagnitude    = m_moiDvCirc;
                plan.burnDuration   = m_moiBurnDur;
                plan.retrogradeBurn = true;
                plan.slewOnArm      = false;   // pilot orients manually; retro asserted at T-10s
                if (m_eventQueue) m_eventQueue->cancelByName("MOI");
                m_moiBurnCtrl.arm(plan, m_autopilot, m_eventQueue);
            }
            return;
        }
        return;
    }
    if (m_page == 4) return;
    if (m_page == 3) {
        if (slot == 4) { m_page = 5; return; }
        return;
    }
    if (m_page == 2) {
        if (slot == 4) m_page = 3;
        return;
    }
    if (m_page == 1) {
        if (slot == 4) m_page = 2;
        return;
    }
    const double depMid = (m_params.t0 + m_params.t1) * 0.5;
    const double tofMid = (m_params.tofMin + m_params.tofMax) * 0.5;
    double depHalf = (m_params.t1 - m_params.t0) * 0.5;
    double tofHalf = (m_params.tofMax - m_params.tofMin) * 0.5;

    switch (slot) {
    case 0:  // COMP or INFO — compute grid, or open detail page if cell selected
        if (m_selDep >= 0 && m_selTof >= 0 && m_hasData) {
            resolveSelected();
            if (m_detail.valid) {
                m_page = 1;
                if (m_eventQueue) {
                    if (m_detail.depET > m_currentET)
                        m_eventQueue->schedule("TMI", m_detail.depET);

                    // Standard trajectory correction waypoints.
                    // Skip any that are in the past or within 6 h of the previous one;
                    // this handles short-TOF transfers where late waypoints predate early ones.
                    struct MccWpt { const char* name; double et; };
                    const double arrET = m_detail.arrET;
                    const double depET = m_detail.depET;
                    const double tof   = m_detail.tofSec;
                    // MCC-1 scales with TOF so the Lambert solution has time to
                    // converge: 1/60 of TOF, minimum 7 days.  This gives ~7 d for
                    // Mars transfers and ~25 d for Jupiter.
                    const double mcc1Lead = std::max(7.0 * kDay, tof / 60.0);
                    const MccWpt wpts[] = {
                        { "MCC-1", depET  + mcc1Lead    },  // early post-departure
                        { "MCC-2", depET  + tof  * 0.5  },  // mid-course
                        { "MCC-3", arrET - 30.0 * kDay  },  // arrival −30 d
                        { "MCC-4", arrET -  7.0 * kDay  },  // arrival −7 d
                        { "MCC-5", arrET -  1.0 * kDay  },  // arrival −24 h
                    };
                    constexpr double kMinSep = 6.0 * 3600.0;
                    double prevET = depET;
                    for (const auto& w : wpts) {
                        if (w.et > m_currentET
                            && w.et > prevET  + kMinSep
                            && w.et < arrET   - kMinSep) {
                            m_eventQueue->schedule(w.name, w.et);
                            prevET = w.et;
                        }
                    }

                    m_eventQueue->schedule("MOI", arrET);
                }
            }
        } else {
            compute();
        }
        break;
    case 1:
        depHalf = std::max(depHalf * 0.5, 30.0 * kDay);
        m_params.t0 = depMid - depHalf;
        m_params.t1 = depMid + depHalf;
        break;
    case 2:
        depHalf *= 2.0;
        m_params.t0 = depMid - depHalf;
        m_params.t1 = depMid + depHalf;
        break;
    case 3:
        tofHalf = std::max(tofHalf * 0.5, 15.0 * kDay);
        m_params.tofMin = tofMid - tofHalf;
        m_params.tofMax = tofMid + tofHalf;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    case 4:  // RNG> — double TOF range
        tofHalf *= 2.0;
        m_params.tofMin = tofMid - tofHalf;
        m_params.tofMax = tofMid + tofHalf;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
void TransferMFD::resolveSelected()
{
    m_detail = {};
    if (!m_hasData || m_selDep < 0 || m_selTof < 0) return;

    const double depET  = m_data.depET(m_selDep);
    const double tofSec = m_data.tofS (m_selTof);
    const double arrET  = depET + tofSec;

    astro::PosState dep, arr;
    try {
        astro::Spice().getRelativeGeometricState(
            m_params.departureBody, m_params.centralBody,
            astro::EphemerisTime(depET), dep, kEclipJ2000);
        astro::Spice().getRelativeGeometricState(
            m_params.arrivalBody, m_params.centralBody,
            astro::EphemerisTime(arrET), arr, kEclipJ2000);
    } catch (...) { return; }

    double mu = m_params.muCentral;
    if (mu <= 0.0)
        astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", mu);

    glm::dvec3 vDep, vArr;
    if (!spacecraft::solveLambert(mu, dep.r, arr.r, tofSec, true, vDep, vArr))
        return;

    glm::dvec3 vInf = vDep - dep.v;
    m_detail.valid    = true;
    m_detail.depET    = depET;
    m_detail.arrET    = arrET;
    m_detail.tofSec   = tofSec;
    m_detail.dv1      = static_cast<float>(glm::length(vInf));
    m_detail.dv2      = static_cast<float>(glm::length(vArr - arr.v));
    m_detail.c3       = static_cast<float>(glm::dot(vInf, vInf));
    m_detail.depPos   = dep.r;
    m_detail.arrPos   = arr.r;
    m_detail.vDepBody = dep.v;
    m_detail.vArrBody = arr.v;
    m_detail.vDep     = vDep;
    m_detail.vArr     = vArr;
}

// ---------------------------------------------------------------------------
void TransferMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    if (m_page == 5) { renderArrival   (dl, origin, size); return; }
    if (m_page == 4) { renderBodySelect(dl, origin, size); return; }
    if (m_page == 3) { renderCoasting  (dl, origin, size); return; }
    if (m_page == 2) { renderDeparture (dl, origin, size); return; }
    if (m_page == 1) { renderDetail    (dl, origin, size); return; }
    renderPorkchop(dl, origin, size);
}

// ---------------------------------------------------------------------------
// Page 4: body selection.
// Shows heliocentric orbits of all planets; highlights DEP (blue) and ARR (red).
// Buttons cycle through planets independently for each role.
// ---------------------------------------------------------------------------
void TransferMFD::renderBodySelect(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kDepCol = IM_COL32( 60, 140, 255, 255);   // blue  — departure
    const ImU32 kArrCol = IM_COL32(220,  80,  50, 255);   // red   — arrival
    const ImU32 kOrbit  = IM_COL32(80,  80,  80, 160);    // dim   — other planets

    double muSun = 0.0;
    try { astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", muSun); }
    catch (...) {}
    if (muSun <= 0.0) muSun = 1.32712440018e11;

    // ---- Orbit diagram: all planets ----
    OrbitDiagram diag;

    for (int i = 0; i < static_cast<int>(m_bodies.size()); ++i) {
        try {
            astro::PosState ps;
            astro::Spice().getRelativeGeometricState(
                m_bodies[i].naifId, m_params.centralBody,
                astro::EphemerisTime(m_currentET), ps, kEclipJ2000);
            glm::dvec3 r(ps.r.x, ps.r.y, ps.r.z);
            glm::dvec3 v(ps.v.x, ps.v.y, ps.v.z);

            ImU32 col;
            bool isDepIdx = (i == m_depPlanetIdx);
            bool isArrIdx = (i == m_arrPlanetIdx);
            if (isDepIdx && isArrIdx)
                col = IM_COL32(180, 100, 200, 220);   // same body: purple
            else if (isDepIdx)
                col = IM_COL32( 60, 140, 255, 180);
            else if (isArrIdx)
                col = IM_COL32(220,  80,  50, 180);
            else
                col = kOrbit;

            diag.addOrbit(r, v, muSun, col, "", false, false);

            ImU32 mCol = isDepIdx ? kDepCol : (isArrIdx ? kArrCol : IM_COL32(120,120,120,200));
            char label[4];
            std::snprintf(label, sizeof(label), "%.3s", m_bodies[i].shortName.c_str());
            diag.addMarker(r, mCol, label);
        } catch (...) {}
    }

    OrbitDiagram::CentralBody sun;
    sun.rimColour  = IM_COL32(255, 210, 60, 200);
    sun.axisColour = IM_COL32(255, 210, 60, 80);
    sun.drawAxes   = false;
    diag.setCentralBody(sun);

    diag.render(dl, origin, size, &m_bodySelViewRot);

    // ---- Text overlay ----
    const float pad   = 4.0f;
    const float lineH = 11.0f;
    const float bx    = origin.x + pad;
    float        by   = origin.y + pad;

    auto txt = [&](ImU32 col, const char* fmt, ...) {
        char buf[80];
        va_list ap; va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddRectFilled({bx - 1, by - 1},
                          {bx + tsz.x + 2, by + tsz.y + 1},
                          IM_COL32(0, 0, 0, 160), 2.0f);
        dl->AddText({bx, by}, col, buf);
        by += lineH;
    };

    txt(kGreen, "BODY SELECT  [OK to confirm]");
    by += pad;
    txt(kDepCol, "DEP: %s  [DEP< DEP>]", m_depBodyName.c_str());
    txt(kArrCol, "ARR: %s  [ARR< ARR>]", m_arrBodyName.c_str());
    by += pad;

    // Show Hohmann transfer time estimate
    try {
        astro::PosState depS, arrS;
        astro::Spice().getRelativeGeometricState(
            m_params.departureBody, m_params.centralBody,
            astro::EphemerisTime(m_currentET), depS, kEclipJ2000);
        astro::Spice().getRelativeGeometricState(
            m_params.arrivalBody, m_params.centralBody,
            astro::EphemerisTime(m_currentET), arrS, kEclipJ2000);
        double rDep = glm::length(glm::dvec3(depS.r.x, depS.r.y, depS.r.z));
        double rArr = glm::length(glm::dvec3(arrS.r.x, arrS.r.y, arrS.r.z));
        double aXfer = (rDep + rArr) * 0.5;
        double tHoh = M_PI * std::sqrt(aXfer * aXfer * aXfer / muSun) / kDay;
        txt(kDim, "Hohmann TOF ~%.0f days", tHoh);
    } catch (...) {}
}

// ---------------------------------------------------------------------------
void TransferMFD::renderPorkchop(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen    = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim      = IM_COL32(  0, 140,  50, 140);
    const ImU32 kWhite    = IM_COL32(255, 255, 255, 220);
    const ImU32 kYellow   = IM_COL32(255, 220,   0, 220);
    const ImU32 kNoData   = IM_COL32( 40,  40,  40, 200);

    const float pad    = 4.0f;
    const float labelH = 11.0f;   // line height for axis labels
    // Info panel: 3 text lines + gap from grid bottom tick labels (labelH).
    const float infoH  = labelH * 3.0f + labelH + pad;

    // Grid area.
    const float gx0 = origin.x + pad;
    const float gy0 = origin.y + pad;
    const float gx1 = origin.x + size.x - pad;
    const float gy1 = origin.y + size.y - infoH - pad;
    const float gw  = gx1 - gx0;
    const float gh  = gy1 - gy0;

    // ---- "COMPUTING" overlay ----
    if (m_computing) {
        const char* msg = "COMPUTING...";
        ImVec2 tsz = ImGui::CalcTextSize(msg);
        dl->AddText({ gx0 + (gw - tsz.x) * 0.5f,
                      gy0 + (gh - tsz.y) * 0.5f }, kGreen, msg);
        return;
    }

    // ---- No data yet ----
    if (!m_hasData) {
        char msg1buf[64];
        std::snprintf(msg1buf, sizeof(msg1buf), "XFER: %s -> %s",
                      m_depBodyName.c_str(), m_arrBodyName.c_str());
        const char* msg1 = msg1buf;
        const char* msg2 = m_error.empty() ? "Press COMP to compute" : m_error.c_str();
        ImU32       col2 = m_error.empty() ? kDim : IM_COL32(255, 80, 80, 220);
        ImVec2 s1 = ImGui::CalcTextSize(msg1);
        ImVec2 s2 = ImGui::CalcTextSize(msg2);
        float cy = gy0 + (gh - labelH * 2.0f - pad) * 0.5f;
        dl->AddText({ gx0 + (gw - s1.x) * 0.5f,       cy             }, kGreen, msg1);
        dl->AddText({ gx0 + (gw - s2.x) * 0.5f, cy + labelH + pad }, col2,   msg2);
        return;
    }

    const int nDep = m_data.params.nDep;
    const int nTof = m_data.params.nTof;
    const float cellW = gw / static_cast<float>(nDep);
    const float cellH = gh / static_cast<float>(nTof);

    // ---- Draw grid cells ----
    // Remap ΔV: clamp at dvMin + 3*(dvMin) to saturate the scale sensibly.
    const float dvLo  = m_data.dvMin;
    const float dvHi  = std::min(m_data.dvMax, dvLo * 4.0f + 3.0f);  // km/s cap
    const float dvRng = (dvHi > dvLo) ? (dvHi - dvLo) : 1.0f;

    for (int iDep = 0; iDep < nDep; ++iDep) {
        for (int iTof = 0; iTof < nTof; ++iTof) {
            float dv  = m_data.get(m_data.dvTotal, iDep, iTof);
            float cx0 = gx0 + iDep       * cellW;
            float cy0 = gy0 + (nTof - 1 - iTof) * cellH;   // TOF increases upward

            ImU32 col;
            if (dv >= spacecraft::PorkchopData::kNoSolution * 0.5f) {
                col = kNoData;
            } else {
                float t = (dv - dvLo) / dvRng;
                col = dvColor(t);
            }
            dl->AddRectFilled({ cx0, cy0 }, { cx0 + cellW, cy0 + cellH }, col);
        }
    }

    // ---- Contour lines (marching squares) ----
    {
        // ΔV iso-levels to draw (km/s).
        static const float kLevels[] = { 6.f, 7.f, 8.f, 9.f, 10.f, 12.f, 15.f };
        static const int   kNLev     = static_cast<int>(std::size(kLevels));

        // Marching squares lookup: {eA,eB,eC,eD} — draw segment(eA→eB) and optional (eC→eD).
        // Cases 5 & 10 are saddles, handled separately.
        static const int8_t kMS[16][4] = {
            {-1,-1,-1,-1}, // 0  0000
            { 3, 0,-1,-1}, // 1  0001  v0 above
            { 0, 1,-1,-1}, // 2  0010  v1 above
            { 3, 1,-1,-1}, // 3  0011  v0+v1
            { 1, 2,-1,-1}, // 4  0100  v2
            { 0, 0, 0, 0}, // 5  0101  saddle
            { 0, 2,-1,-1}, // 6  0110  v1+v2
            { 3, 2,-1,-1}, // 7  0111  v0+v1+v2
            { 2, 3,-1,-1}, // 8  1000  v3
            { 2, 0,-1,-1}, // 9  1001  v0+v3
            { 0, 0, 0, 0}, //10  1010  saddle
            { 2, 1,-1,-1}, //11  1011  v0+v1+v3
            { 1, 3,-1,-1}, //12  1100  v2+v3
            { 1, 0,-1,-1}, //13  1101  v0+v2+v3
            { 0, 3,-1,-1}, //14  1110  v1+v2+v3
            {-1,-1,-1,-1}, //15  1111
        };

        // Interpolated screen position of threshold crossing on one edge of a square.
        // Square: data-space corners (i,j),(i+1,j),(i+1,j+1),(i,j+1) = v0,v1,v2,v3.
        // Edge: 0=bottom(v0-v1), 1=right(v1-v2), 2=top(v2-v3), 3=left(v3-v0).
        auto edgePt = [&](int i, int j, int edge, float lev,
                          float v0, float v1, float v2, float v3) -> ImVec2 {
            float bx = gx0 + (i + 0.5f) * cellW;
            float by = gy0 + (nTof - 0.5f - j) * cellH;
            float tx = bx + cellW;
            float ty = by - cellH;   // ty < by: higher j → higher on screen → smaller y
            switch (edge) {
            case 0: { float t=(lev-v0)/(v1-v0); return {bx+t*cellW, by         }; }
            case 1: { float t=(lev-v1)/(v2-v1); return {tx,          by-t*cellH }; }
            case 2: { float t=(lev-v2)/(v3-v2); return {tx-t*cellW,  ty         }; }
            case 3: { float t=(lev-v3)/(v0-v3); return {bx,          ty+t*cellH }; }
            default: return {bx, by};
            }
        };

        const float kNS = spacecraft::PorkchopData::kNoSolution * 0.5f;

        // Track one label point per level (where contour last crosses a right-side edge).
        struct LabelPt { float x, y; bool valid = false; };
        std::vector<LabelPt> lblPts(kNLev);

        for (int li = 0; li < kNLev; ++li) {
            const float lev = kLevels[li];
            if (lev < dvLo - 0.5f || lev > dvHi * 1.5f) continue;

            const ImU32 col = IM_COL32(255, 255, 255, 110);

            for (int i = 0; i < nDep - 1; ++i) {
                for (int j = 0; j < nTof - 1; ++j) {
                    float v0 = m_data.get(m_data.dvTotal, i,   j  );
                    float v1 = m_data.get(m_data.dvTotal, i+1, j  );
                    float v2 = m_data.get(m_data.dvTotal, i+1, j+1);
                    float v3 = m_data.get(m_data.dvTotal, i,   j+1);
                    if (v0>kNS || v1>kNS || v2>kNS || v3>kNS) continue;

                    int ci = ((v0>lev)?1:0)|((v1>lev)?2:0)|((v2>lev)?4:0)|((v3>lev)?8:0);
                    if (ci == 0 || ci == 15) continue;

                    int8_t e[4];
                    if (ci == 5 || ci == 10) {
                        // Disambiguate saddle by centre value.
                        bool ca = ((v0+v1+v2+v3)*0.25f > lev);
                        if (ci == 5) {
                            // v0,v2 above; v1,v3 below
                            if (!ca) { e[0]=3;e[1]=0;e[2]=1;e[3]=2; }
                            else     { e[0]=0;e[1]=1;e[2]=2;e[3]=3; }
                        } else {
                            // v1,v3 above; v0,v2 below
                            if (!ca) { e[0]=0;e[1]=1;e[2]=2;e[3]=3; }
                            else     { e[0]=3;e[1]=0;e[2]=1;e[3]=2; }
                        }
                    } else {
                        e[0]=kMS[ci][0]; e[1]=kMS[ci][1];
                        e[2]=kMS[ci][2]; e[3]=kMS[ci][3];
                    }

                    if (e[0]>=0 && e[1]>=0) {
                        ImVec2 pa = edgePt(i,j,e[0],lev,v0,v1,v2,v3);
                        ImVec2 pb = edgePt(i,j,e[1],lev,v0,v1,v2,v3);
                        dl->AddLine(pa, pb, col, 1.0f);
                        // Track label position: prefer middle column, accept any
                        if (!lblPts[li].valid || i > lblPts[li].x) {
                            lblPts[li] = {float(i), (pa.y+pb.y)*0.5f, true};
                        }
                    }
                    if (e[2]>=0 && e[3]>=0) {
                        ImVec2 pc = edgePt(i,j,e[2],lev,v0,v1,v2,v3);
                        ImVec2 pd = edgePt(i,j,e[3],lev,v0,v1,v2,v3);
                        dl->AddLine(pc, pd, col, 1.0f);
                    }
                }
            }
        }

        // Draw inline labels at the rightmost crossing of each level.
        for (int li = 0; li < kNLev; ++li) {
            if (!lblPts[li].valid) continue;
            char buf[6]; std::snprintf(buf, sizeof(buf), "%.0f", kLevels[li]);
            ImVec2 tsz = ImGui::CalcTextSize(buf);
            float lx = gx0 + (lblPts[li].x + 1.5f) * cellW - tsz.x * 0.5f;
            float ly = lblPts[li].y - tsz.y * 0.5f;
            // Small dark backing so text is readable over the colour map.
            dl->AddRectFilled({lx-1,ly-1}, {lx+tsz.x+1,ly+tsz.y+1},
                              IM_COL32(0,0,0,160));
            dl->AddText({lx, ly}, IM_COL32(255,255,200,220), buf);
        }
    }

    // ---- Mouse hover ----
    const ImVec2 mp = ImGui::GetMousePos();
    const bool inGrid = mp.x >= gx0 && mp.x < gx1 && mp.y >= gy0 && mp.y < gy1;
    int hDep = -1, hTof = -1;
    if (inGrid) {
        hDep = static_cast<int>((mp.x - gx0) / cellW);
        hTof = nTof - 1 - static_cast<int>((mp.y - gy0) / cellH);
        hDep = std::clamp(hDep, 0, nDep - 1);
        hTof = std::clamp(hTof, 0, nTof - 1);

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            m_selDep = hDep;
            m_selTof = hTof;
        }
    }

    // ---- Selection crosshair ----
    if (m_selDep >= 0 && m_selTof >= 0) {
        float sx = gx0 + (m_selDep + 0.5f) * cellW;
        float sy = gy0 + (nTof - 1 - m_selTof + 0.5f) * cellH;
        const float cs = 5.0f;
        dl->AddLine({ sx - cs, sy }, { sx + cs, sy }, kWhite, 1.0f);
        dl->AddLine({ sx, sy - cs }, { sx, sy + cs }, kWhite, 1.0f);
    }

    // ---- Hover crosshair (dimmer) ----
    if (inGrid) {
        float sx = gx0 + (hDep + 0.5f) * cellW;
        float sy = gy0 + (nTof - 1 - hTof + 0.5f) * cellH;
        dl->AddLine({ gx0, sy }, { gx1, sy }, IM_COL32(255,255,255,50), 0.5f);
        dl->AddLine({ sx, gy0 }, { sx, gy1 }, IM_COL32(255,255,255,50), 0.5f);
    }

    // ---- "Today" marker — vertical line at current simulation ET ----
    {
        const double t0 = m_data.params.t0;
        const double t1 = m_data.params.t1;
        if (m_currentET >= t0 && m_currentET <= t1) {
            const float nx = gx0 + static_cast<float>((m_currentET - t0) / (t1 - t0)) * gw;
            dl->AddLine({ nx, gy0 }, { nx, gy1 },
                        IM_COL32(80, 220, 255, 180), 1.0f);
            const char* nowLabel = "NOW";
            ImVec2 tsz = ImGui::CalcTextSize(nowLabel);
            // Label just above the bottom of the grid so it doesn't collide with ticks.
            dl->AddRectFilled({ nx - 1.0f, gy1 - tsz.y - 3.0f },
                              { nx + tsz.x + 2.0f, gy1 - 1.0f },
                              IM_COL32(0, 0, 0, 140));
            dl->AddText({ nx + 1.0f, gy1 - tsz.y - 2.0f },
                        IM_COL32(80, 220, 255, 220), nowLabel);
        }
    }

    // ---- Axis ticks ----
    // Departure: 5 ticks along bottom
    for (int i = 0; i <= 4; ++i) {
        float fx  = gx0 + gw * i / 4.0f;
        double et = m_data.params.t0 + (m_data.params.t1 - m_data.params.t0) * i / 4.0;
        std::string ts = astro::EphemerisTime(et).toISOUTCString(0);
        // Show only year-month for brevity
        const char* label = ts.size() >= 7 ? ts.c_str() : "?";
        char buf[8]; std::snprintf(buf, sizeof(buf), "%.7s", label);
        dl->AddLine({ fx, gy1 }, { fx, gy1 + 3.0f }, kDim, 1.0f);
        dl->AddText({ fx - 14.0f, gy1 + 2.0f }, kDim, buf);
    }
    // TOF: 3 ticks on left
    for (int i = 0; i <= 3; ++i) {
        float fy   = gy1 - gh * i / 3.0f;
        double tof = m_data.params.tofMin
                   + (m_data.params.tofMax - m_data.params.tofMin) * i / 3.0;
        int tofDays = static_cast<int>(tof / kDay + 0.5);
        char buf[8]; std::snprintf(buf, sizeof(buf), "%dd", tofDays);
        dl->AddLine({ gx0 - 3.0f, fy }, { gx0, fy }, kDim, 1.0f);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ gx0 - tsz.x - 2.0f, fy - tsz.y * 0.5f }, kDim, buf);
    }

    // ---- Info panel ----
    const float infoY = gy1 + labelH + pad;   // below tick labels
    int di = (inGrid ? hDep : m_selDep);
    int ti = (inGrid ? hTof : m_selTof);

    if (di >= 0 && ti >= 0) {
        double depET  = m_data.depET(di);
        double tofSec = m_data.tofS(ti);
        float  dv1    = m_data.get(m_data.dv1,    di, ti);
        float  dv2    = m_data.get(m_data.dv2,    di, ti);
        float  dvTot  = m_data.get(m_data.dvTotal, di, ti);

        std::string depStr = astro::EphemerisTime(depET).toISOUTCString(0);
        int tofDays = static_cast<int>(tofSec / kDay + 0.5);

        char buf[64];
        std::snprintf(buf, sizeof(buf), "DEP  %.10s", depStr.c_str());
        dl->AddText({ origin.x + pad, infoY }, kGreen, buf);

        std::snprintf(buf, sizeof(buf), "TOF  %dd", tofDays);
        dl->AddText({ origin.x + pad, infoY + labelH }, kGreen, buf);

        if (dvTot < spacecraft::PorkchopData::kNoSolution * 0.5f) {
            std::snprintf(buf, sizeof(buf), "DV1 %.2f  DV2 %.2f  TOT %.2f km/s",
                          dv1, dv2, dvTot);
            dl->AddText({ origin.x + pad, infoY + 2.0f * labelH }, kYellow, buf);
        } else {
            dl->AddText({ origin.x + pad, infoY + 2.0f * labelH }, kDim, "no solution");
        }
    } else {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "dVmin %.2f km/s", m_data.dvMin);
        dl->AddText({ origin.x + pad, infoY }, kGreen, buf);
        dl->AddText({ origin.x + pad, infoY + labelH }, kDim, "hover/click to select");
    }
}


// ---------------------------------------------------------------------------
void TransferMFD::renderDetail(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan   = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange = IM_COL32(255, 140,   0, 220);

    const float pad   = 4.0f;
    const float lineH = 11.0f;
    const float cx    = origin.x + pad;
    float y           = origin.y + pad;

    if (!m_detail.valid) {
        dl->AddText({cx, y}, kDim, "No transfer selected");
        return;
    }

    // Mu — needed throughout.
    double mu = m_params.muCentral;
    if (mu <= 0.0) {
        try { astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", mu); }
        catch (...) {}
    }

    // ---- v∞ and burn geometry ----
    glm::dvec3 vInf    = m_detail.vDep - m_detail.vDepBody;
    double     vInfMag = glm::length(vInf);

    const double muDep   = m_muDep;
    const double rPark   = m_depBodyRadius + kParkAlts[m_parkIdx];
    const double altKm   = kParkAlts[m_parkIdx];
    const double vCirc   = std::sqrt(muDep / rPark);
    const double vPeri   = std::sqrt(m_detail.c3 + 2.0 * muDep / rPark);
    const double dvTMI   = vPeri - vCirc;                      // TMI burn ΔV

    // Inclination of v∞ above ecliptic = angle between v∞ and ecliptic plane.
    // sin(inc) = vInf.z / |vInf|  (ecliptic frame: z is out-of-plane)
    double vInfInc = 0.0;
    if (vInfMag > 1e-6)
        vInfInc = std::asin(std::clamp(vInf.z / vInfMag, -1.0, 1.0)) * 180.0 / M_PI;

    // ---- Text rows ----
    auto row = [&](ImU32 col, const char* fmt, ...) {
        char buf[80];
        va_list ap; va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        dl->AddText({cx, y}, col, buf);
        y += lineH;
    };

    std::string nowStr = astro::EphemerisTime(m_currentET).toISOUTCString(0);
    std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
    std::string arrStr = astro::EphemerisTime(m_detail.arrET).toISOUTCString(0);
    int tofDays    = static_cast<int>(m_detail.tofSec / kDay + 0.5);
    char tplusBuf[16];
    fmtTplus(m_detail.depET - m_currentET, tplusBuf, sizeof(tplusBuf));

    row(kDim,    "NOW  %.10s", nowStr.c_str());
    row(kGreen,  "DEP  %.10s  (%s)", depStr.c_str(), tplusBuf);
    row(kGreen,  "ARR  %.10s", arrStr.c_str());
    row(kGreen,  "TOF  %d days", tofDays);
    y += pad;
    row(kYellow, "DV1  %.3f km/s", m_detail.dv1);
    row(kYellow, "DV2  %.3f km/s", m_detail.dv2);
    row(kYellow, "TOT  %.3f km/s", m_detail.dv1 + m_detail.dv2);
    y += pad;
    row(kCyan,   "C3   %.2f km2/s2", m_detail.c3);

    if (vInfMag > 1e-6) {
        glm::dvec3 vn  = vInf / vInfMag;
        double lon = std::atan2(vn.y, vn.x) * 180.0 / M_PI;
        if (lon < 0.0) lon += 360.0;
        double lat = std::asin(std::clamp(vn.z, -1.0, 1.0)) * 180.0 / M_PI;
        row(kCyan, "VINF LON %.0f  LAT %.1f", lon, lat);
    }
    y += pad;
    row(kDim,    "TMI  alt %.0f km", altKm);
    row(kGreen,  "  Vcirc  %.3f km/s", vCirc);
    row(kGreen,  "  Vperi  %.3f km/s", vPeri);
    row(kYellow, "  dV-TMI %.3f km/s", dvTMI);
    if (std::abs(vInfInc) > 0.5)
        row(kOrange, "  inc    %.1f deg above ecl", vInfInc);

    // ---- Orbit diagram via OrbitDiagram ----
    const float diagH = size.y - (y - origin.y) - pad;
    if (diagH < 20.0f || mu <= 0.0) return;

    // Build transfer arc as a 3D polyline.
    OrbitDiagram::Arc transferArc;
    transferArc.colour    = kYellow;
    transferArc.thickness = 1.5f;
    {
        glm::dvec3 h    = glm::cross(m_detail.depPos, m_detail.vDep);
        double     hMag = glm::length(h);
        if (hMag > 1e-6) {
            glm::dvec3 hn      = h / hMag;
            double     p_      = hMag * hMag / mu;
            glm::dvec3 ev      = glm::cross(m_detail.vDep, h) / mu
                               - glm::normalize(m_detail.depPos);
            double     ecc     = glm::length(ev);
            glm::dvec3 periDir = (ecc > 1e-9)
                               ? glm::normalize(ev) : glm::normalize(m_detail.depPos);
            glm::dvec3 qDir    = glm::cross(hn, periDir);

            auto nu_of = [&](const glm::dvec3& r) -> double {
                double cosnu = glm::dot(periDir, glm::normalize(r));
                cosnu = std::clamp(cosnu, -1.0, 1.0);
                double nu = std::acos(cosnu);
                if (glm::dot(glm::cross(periDir, r), hn) < 0.0) nu = -nu;
                return nu;
            };
            double nu1 = nu_of(m_detail.depPos);
            double nu2 = nu_of(m_detail.arrPos);
            if (nu2 < nu1) nu2 += 2.0 * M_PI;

            for (int k = 0; k <= 80; ++k) {
                double nu = nu1 + (nu2 - nu1) * k / 80.0;
                double r_ = p_ / (1.0 + ecc * std::cos(nu));
                transferArc.pts.push_back(
                    r_ * (std::cos(nu) * periDir + std::sin(nu) * qDir));
            }
        }
    }

    OrbitDiagram diag;
    diag.addOrbit(m_detail.depPos, m_detail.vDepBody, mu,
                  IM_COL32(60, 140, 255, 90), "", true, false);
    diag.addOrbit(m_detail.arrPos, m_detail.vArrBody, mu,
                  IM_COL32(200, 80, 50, 90),  "", true, false);
    diag.addArc(transferArc);
    diag.addMarker(m_detail.depPos, IM_COL32( 60,140,255,255), m_depBodyName.substr(0,1).c_str());
    diag.addMarker(m_detail.arrPos, IM_COL32(200, 80, 50,255), m_arrBodyName.substr(0,1).c_str());
    if (vInfMag > 1e-6)
        diag.addArrow(m_detail.depPos, vInf / vInfMag, 14.0f, kOrange);

    diag.render(dl, { origin.x, y + pad }, { size.x, diagH }, &m_detailViewRot);
}

// ---------------------------------------------------------------------------
// Page 2: geocentric departure burn planning.
//
// Geometry:
//   - V∞  = vDep_helio − vEarth_helio  (direction same in any inertial frame)
//   - Target orbit plane: closest plane containing V∞
//     hTarget = normalize( hCurrent − (hCurrent·V∞̂)·V∞̂ )
//   - AN of current orbit w.r.t. target plane:  cross(hTarget, hCurrent)
//   - Optimal burn TA ν*: maximise v(ν)·V∞_proj
//     ν* = atan2(−(V∞_proj·periDir), V∞_proj·qDir)
//   - Plane error = asin(|V∞̂·hCurrent|)
//
// Layout: diagram fills full area; all text overlaid with dark backing boxes.
// ---------------------------------------------------------------------------
void TransferMFD::renderDeparture(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan   = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange = IM_COL32(255, 140,   0, 220);
    const ImU32 kRed    = IM_COL32(255,  60,  60, 220);
    const ImU32 kBacking = IM_COL32(0, 0, 0, 175);

    // -----------------------------------------------------------------------
    // Guard conditions — need valid transfer and ship state
    // -----------------------------------------------------------------------
    if (!m_detail.valid) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim, "No transfer selected");
        return;
    }
    const double rShip   = glm::length(m_shipR);
    const double vShipMag = glm::length(m_shipV);
    if (rShip < 100.0 || vShipMag < 0.01) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim, "No ship state");
        return;
    }

    // -----------------------------------------------------------------------
    // Orbital geometry — current parking orbit
    // -----------------------------------------------------------------------
    const double mu   = m_muDep;
    const double kRE  = m_depBodyRadius;

    glm::dvec3 h    = glm::cross(m_shipR, m_shipV);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) { dl->AddText({origin.x+4,origin.y+4}, kDim, "Degenerate orbit"); return; }

    glm::dvec3 hHat    = h / hMag;
    double     p_orb   = hMag * hMag / mu;
    glm::dvec3 ev      = glm::cross(m_shipV, h) / mu - glm::normalize(m_shipR);
    double     ecc     = glm::length(ev);
    glm::dvec3 periDir = (ecc > 1e-6) ? glm::normalize(ev) : glm::normalize(m_shipR);
    glm::dvec3 qDir    = glm::cross(hHat, periDir);
    double     sma     = (ecc < 1.0) ? p_orb / (1.0 - ecc * ecc) : p_orb;
    double     altKm   = rShip - kRE;

    // Ecliptic orbital elements (ECLIPJ2000: z = ecliptic north, x = vernal equinox)
    double inc_cur  = std::acos(std::clamp(hHat.z, -1.0, 1.0)) * 180.0 / M_PI;
    double raan_cur = std::atan2(hHat.x, -hHat.y) * 180.0 / M_PI;
    if (raan_cur < 0.0) raan_cur += 360.0;

    // Argument of periapsis: angle from node line to periDir in orbit plane
    double nodeLen   = std::sqrt(hHat.x*hHat.x + hHat.y*hHat.y);
    double argpe_cur = 0.0;
    if (nodeLen > 1e-9 && ecc > 1e-6) {
        glm::dvec3 nodeDir(-hHat.y / nodeLen, hHat.x / nodeLen, 0.0);
        argpe_cur = std::atan2(
            glm::dot(periDir, glm::cross(hHat, nodeDir)),
            glm::dot(periDir, nodeDir)) * 180.0 / M_PI;
        if (argpe_cur < 0.0) argpe_cur += 360.0;
    }

    // Current true anomaly
    glm::dvec3 rHat   = glm::normalize(m_shipR);
    double     ta_now = std::atan2(glm::dot(rHat, qDir), glm::dot(rHat, periDir));
    double     taDeg_now = ta_now * 180.0 / M_PI;
    if (taDeg_now < 0.0) taDeg_now += 360.0;

    // -----------------------------------------------------------------------
    // V∞
    // -----------------------------------------------------------------------
    glm::dvec3 vInf    = m_detail.vDep - m_detail.vDepBody;
    double     vInfMag = glm::length(vInf);
    if (vInfMag < 1e-6) { dl->AddText({origin.x+4,origin.y+4}, kDim, "Zero V-inf"); return; }
    glm::dvec3 vInfHat = vInf / vInfMag;

    // -----------------------------------------------------------------------
    // Target orbit plane: closest to current, constrained to contain V∞
    // hTarget = normalize( hHat − (hHat·V∞̂)·V∞̂ )
    // -----------------------------------------------------------------------
    glm::dvec3 hProj   = hHat - glm::dot(hHat, vInfHat) * vInfHat;
    double     hProjL  = glm::length(hProj);
    glm::dvec3 hTarget = (hProjL > 1e-9) ? (hProj / hProjL)
                       : glm::normalize(glm::cross(vInfHat, glm::dvec3(0,0,1)));

    double inc_tgt  = std::acos(std::clamp(hTarget.z, -1.0, 1.0)) * 180.0 / M_PI;
    double raan_tgt = std::atan2(hTarget.x, -hTarget.y) * 180.0 / M_PI;
    if (raan_tgt < 0.0) raan_tgt += 360.0;
    double dInc = inc_tgt - inc_cur;

    // -----------------------------------------------------------------------
    // Plane error
    // -----------------------------------------------------------------------
    double planeErr = std::asin(std::clamp(std::abs(glm::dot(vInfHat, hHat)), 0.0, 1.0))
                      * 180.0 / M_PI;

    // -----------------------------------------------------------------------
    // AN / DN of current orbit w.r.t. target plane
    // AN direction = cross(hTarget, hCurrent)
    // -----------------------------------------------------------------------
    glm::dvec3 anVec = glm::cross(hTarget, hHat);
    double     anLen = glm::length(anVec);
    bool       hasNodes = (anLen > 1e-9);

    double ta_AN = 0.0, ta_DN = 0.0;
    double taDeg_AN = 0.0, taDeg_DN = 0.0;
    double tToAN = 0.0, tToDN = 0.0;
    glm::dvec3 anPos(0.0), dnPos(0.0);

    if (hasNodes) {
        glm::dvec3 anDir = anVec / anLen;
        glm::dvec3 dnDir = -anDir;

        ta_AN = std::atan2(glm::dot(anDir, qDir), glm::dot(anDir, periDir));
        ta_DN = ta_AN + M_PI;
        // Normalise DN to (-π, π)
        if (ta_DN >  M_PI) ta_DN -= 2.0 * M_PI;

        taDeg_AN = ta_AN * 180.0 / M_PI; if (taDeg_AN < 0.0) taDeg_AN += 360.0;
        taDeg_DN = ta_DN * 180.0 / M_PI; if (taDeg_DN < 0.0) taDeg_DN += 360.0;

        double rAN = p_orb / (1.0 + ecc * std::cos(ta_AN));
        double rDN = p_orb / (1.0 + ecc * std::cos(ta_DN));
        anPos = rAN * (std::cos(ta_AN) * periDir + std::sin(ta_AN) * qDir);
        dnPos = rDN * (std::cos(ta_DN) * periDir + std::sin(ta_DN) * qDir);

        // Time to AN/DN via Kepler (elliptic only)
        if (ecc < 1.0 && sma > 0.0) {
            auto ta2M = [&](double ta) {
                double E = 2.0 * std::atan2(
                    std::sqrt(1.0 - ecc) * std::sin(ta * 0.5),
                    std::sqrt(1.0 + ecc) * std::cos(ta * 0.5));
                return E - ecc * std::sin(E);
            };
            double n_mot = std::sqrt(mu / (sma * sma * sma));
            double M_now = ta2M(ta_now);
            double dM_AN = ta2M(ta_AN) - M_now; if (dM_AN <= 0.0) dM_AN += 2.0 * M_PI;
            double dM_DN = ta2M(ta_DN) - M_now; if (dM_DN <= 0.0) dM_DN += 2.0 * M_PI;
            tToAN = dM_AN / n_mot;
            tToDN = dM_DN / n_mot;
        }
    }

    // -----------------------------------------------------------------------
    // Optimal burn TA
    // -----------------------------------------------------------------------
    // The burn is prograde (along current velocity) at the point in the parking
    // orbit where the resulting escape hyperbola's asymptote aligns with V∞.
    //
    // A prograde burn at orbit angle θ produces an asymptote at angle:
    //   φ_asymptote = θ + ν∞   where ν∞ = acos(-1/e_hyp)
    // (derived from: asymptote_dir = cos(ν∞)·r̂_burn + sin(ν∞)·T_burn,
    //  which expands to angle θ+ν∞ in the (periDir,qDir) frame)
    //
    // Setting φ_asymptote = φ_target:  θ = φ_target − ν∞
    //
    // The previous formula (atan2(-vp,vq) = φ_target−90°) aligned the VELOCITY
    // with V∞ instead of the asymptote — correct only for infinite C3, off by
    // ~60° for C3 ≈ 9 km²/s².
    const double rPark = m_depBodyRadius + kParkAlts[m_parkIdx];
    const double e_hyp  = 1.0 + static_cast<double>(m_detail.c3) * rPark / mu;
    const double nu_inf = std::acos(-1.0 / e_hyp);   // asymptote true anomaly [rad]

    glm::dvec3 vInfProj    = vInf - glm::dot(vInf, hHat) * hHat;
    double     vInfProjMag = glm::length(vInfProj);
    double burnTA = 0.0, burnTADeg = 0.0;
    if (vInfProjMag > 1e-9) {
        double vp = glm::dot(vInfProj, periDir);
        double vq = glm::dot(vInfProj, qDir);
        double phi_target = std::atan2(vq, vp);   // direction of V∞_proj in orbit plane
        burnTA = phi_target - nu_inf;
        while (burnTA >  M_PI) burnTA -= 2.0 * M_PI;
        while (burnTA < -M_PI) burnTA += 2.0 * M_PI;
        burnTADeg = burnTA * 180.0 / M_PI;
        if (burnTADeg < 0.0) burnTADeg += 360.0;
    }
    double rBurn   = p_orb / (1.0 + ecc * std::cos(burnTA));
    double altBurn = rBurn - kRE;
    glm::dvec3 burnPos = rBurn * (std::cos(burnTA) * periDir + std::sin(burnTA) * qDir);
    double vCirc = std::sqrt(mu / rPark);
    double vPeri = std::sqrt(m_detail.c3 + 2.0 * mu / rPark);
    double dvTMI = vPeri - vCirc;

    // -----------------------------------------------------------------------
    // Burn timing
    // -----------------------------------------------------------------------
    // Time from current TA to burn TA via Kepler's equation (elliptic only).
    double timeToBurnPoint = 0.0;
    double orbPeriod       = 0.0;
    if (ecc < 1.0 && sma > 0.0) {
        orbPeriod = 2.0 * M_PI * std::sqrt(sma * sma * sma / mu);
        auto meanAnom = [&](double ta) -> double {
            double tanH  = std::tan(ta * 0.5) * std::sqrt((1.0 - ecc) / (1.0 + ecc));
            double E     = 2.0 * std::atan(tanH);
            return E - ecc * std::sin(E);
        };
        double M_now  = meanAnom(ta_now);
        double M_burn = meanAnom(burnTA);
        double dM = M_burn - M_now;
        if (dM < 0.0) dM += 2.0 * M_PI;   // wrap to next occurrence
        timeToBurnPoint = dM / (2.0 * M_PI) * orbPeriod;
    }

    // Burn duration and remaining ΔV.
    // accel in m/s², dvTMI in km/s → multiply by 1000 for m/s.
    const double accelMs2    = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
    const double burnDuration = dvTMI * 1000.0 / accelMs2;   // seconds for full burn

    // Live remaining ΔV — use C3 (v² - 2μ/r), not instantaneous speed.
    // Speed-based comparison breaks immediately after the burn: the spacecraft
    // moves away from periapsis, slows down due to gravity, and the difference
    // (vPeri - currentSpeed) grows again even though no more burn is needed.
    // C3 = v² - 2μ/r is a conserved orbital quantity that stays constant after
    // the burn, so the comparison is meaningful throughout the coast phase.
    const double currentSpeed = glm::length(m_shipV);  // km/s
    const double rNow         = glm::length(m_shipR);  // km
    const double c3Now        = currentSpeed * currentSpeed - 2.0 * mu / rNow;
    const double c3Req        = m_detail.c3;            // required C3 from Lambert
    // Remaining ΔV expressed as the speed deficit at the current radius.
    // When c3Now >= c3Req the burn is complete and dV-remaining reads zero.
    const double dvRemaining  = (c3Now < c3Req)
        ? std::max(0.0, std::sqrt(c3Req + 2.0 * mu / rNow) - currentSpeed)
        : 0.0;
    const double burnRemaining = dvRemaining * 1000.0 / accelMs2; // seconds left

    // Time to start burn = time to burn point - half burn duration (periapsis-centered).
    const double timeToStartBurn = timeToBurnPoint - burnDuration * 0.5;

    // -----------------------------------------------------------------------
    // HH:MM:SS formatter
    // -----------------------------------------------------------------------
    auto fmtHMS = [](double sec, char* buf, int sz) {
        if (sec < 0.0 || !std::isfinite(sec)) { std::snprintf(buf, sz, "--:--:--"); return; }
        int s = static_cast<int>(sec + 0.5);
        int hh = s / 3600; s %= 3600;
        int mm = s / 60;   s %= 60;
        std::snprintf(buf, sz, "%d:%02d:%02d", hh, mm, s);
    };

    // -----------------------------------------------------------------------
    // Orbit diagram — full area
    // -----------------------------------------------------------------------
    {
        OrbitDiagram diag;

        // Parking orbit with ship dot
        OrbitDiagram::Orbit o;
        o.r = m_shipR; o.v = m_shipV; o.mu = mu;
        o.colour = kGreen; o.showApses = (ecc > 0.005); o.showCurrent = true;
        diag.addOrbit(o);

        // Target orbit plane visualised as a ghosted ring at same SMA
        // (no real velocity, just a fake circular orbit in the target plane to show the plane)
        {
            // pick a periDir perpendicular to hTarget in the target plane
            glm::dvec3 tPeri = glm::normalize(glm::cross(hTarget, hHat));
            if (glm::length(tPeri) < 0.5)
                tPeri = glm::normalize(glm::cross(hTarget, glm::dvec3(1,0,0)));
            OrbitDiagram::Orbit ot;
            double rcirc   = std::sqrt(mu * sma); // circular speed at current SMA
            ot.r     = sma * tPeri;
            ot.v     = std::sqrt(mu / sma) * glm::cross(hTarget, tPeri);
            ot.mu    = mu;
            ot.colour = IM_COL32(60, 180, 255, 60);  // dim blue ghost
            ot.showApses = false; ot.showCurrent = false;
            diag.addOrbit(ot);
        }

        // AN / DN markers
        if (hasNodes) {
            diag.addMarker(anPos, IM_COL32(100, 255, 100, 230), "AN");
            diag.addMarker(dnPos, IM_COL32(255, 100, 100, 230), "DN");
        }

        // Burn point
        diag.addMarker(burnPos, kYellow, "B");

        // V∞ direction arrow from Earth centre
        if (vInfProjMag > 1e-9)
            diag.addArrow({0.0, 0.0, 0.0}, vInfHat, 22.0f, kOrange);

        OrbitDiagram::CentralBody depBody;
        depBody.radiusKm   = kRE;
        depBody.rimColour  = IM_COL32(60, 140, 255, 200);
        depBody.axisColour = IM_COL32(60, 140, 255, 100);
        depBody.drawAxes   = false;
        diag.setCentralBody(depBody);

        diag.render(dl, origin, size, &m_depViewRot);
    }

    // -----------------------------------------------------------------------
    // Text overlay — build lines, then draw with one backing rect
    // -----------------------------------------------------------------------
    const float pad   = 3.0f;
    const float lineH = 11.0f;

    struct Line { ImU32 col; char txt[72]; };
    std::vector<Line> lines;
    auto add = [&](ImU32 col, const char* fmt, ...) {
        Line l; l.col = col;
        va_list ap; va_start(ap, fmt);
        std::vsnprintf(l.txt, sizeof(l.txt), fmt, ap);
        va_end(ap);
        lines.push_back(l);
    };
    auto sep = [&]() { add(0, ""); };  // blank spacer

    // ---- Burn controller status (top of overlay) ----
    {
        const ImU32 kArmed  = IM_COL32(255, 220,   0, 240);
        const ImU32 kExec   = IM_COL32(255,  80,  80, 240);
        const ImU32 kDone   = IM_COL32(  0, 210,  75, 240);
        switch (m_burnCtrl.phase()) {
        case BurnPhase::Armed: {
            double tti = m_burnCtrl.timeToIgnition(m_currentET);
            char hmsBuf[16];
            fmtHMS(tti, hmsBuf, sizeof(hmsBuf));
            add(kArmed, "ARMED  IGN T-%s  [DSARM]", hmsBuf);
            sep();
            break;
        }
        case BurnPhase::PreIgnition: {
            double tti = m_burnCtrl.timeToIgnition(m_currentET);
            char hmsBuf[16];
            if (tti >= 0.0) fmtHMS(tti, hmsBuf, sizeof(hmsBuf));
            else std::snprintf(hmsBuf, sizeof(hmsBuf), "HOLD");
            add(kArmed, "PRE-IGN  T-%s  PROGRADE CHECK  [DSARM]", hmsBuf);
            sep();
            break;
        }
        case BurnPhase::Executing:
            add(kExec, "EXECUTING BURN  [DSARM]");
            sep();
            break;
        case BurnPhase::Complete:
            add(kDone, "BURN COMPLETE (autopilot)");
            sep();
            break;
        default: break;
        }
    }

    // Header — dates and V∞
    {
        std::string nowStr = astro::EphemerisTime(m_currentET).toISOUTCString(0);
        std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
        char tplusBuf2[16];
        fmtTplus(m_detail.depET - m_currentET, tplusBuf2, sizeof(tplusBuf2));
        add(kDim,   "NOW  %.10s", nowStr.c_str());
        add(kGreen, "DEP  %.10s  (%s)", depStr.c_str(), tplusBuf2);
    }
    add(kCyan,  "V-inf %.3f km/s  C3 %.1f km2/s2", vInfMag, m_detail.c3);
    sep();

    // Current orbit
    add(kCyan,   "CURRENT ORBIT");
    add(kGreen,  " i %.1f  Ω %.1f  ω %.1f  e %.4f",
                 inc_cur, raan_cur, argpe_cur, ecc);
    add(kGreen,  " TA %.1f  Alt %.0f km  SMA %.0f km",
                 taDeg_now, altKm, sma);
    sep();

    // Plane-change cost at current altitude:  dV_pc = 2·v_c·sin(Δi/2)
    const double sinHalfDi    = std::sin(planeErr * M_PI / 180.0 * 0.5);
    const double vLEO         = std::sqrt(mu / rShip);
    const double dvPCdirect   = 2.0 * vLEO * sinHalfDi;
    const double thrustN      = m_mainThrustN;
    const double massKg       = m_shipMass;
    const double accel        = thrustN / massKg;   // m/s²

    // Target plane
    ImU32 tgtCol = (planeErr < 0.5) ? kGreen : (planeErr < 5.0) ? kOrange : kRed;
    add(kCyan,   "TARGET PLANE  (contains V-inf)");
    add(kGreen,  " i %.1f  Ω %.1f", inc_tgt, raan_tgt);
    add(tgtCol,  " Di %.1f deg  Perr %.1f deg%s",
                 dInc, planeErr, planeErr > 5.0 ? " NEED CHG" : "");
    if (planeErr > 0.05) {
        double burnTimeDirect = dvPCdirect * 1000.0 / accel;
        char bufD[12]; fmtHMS(burnTimeDirect, bufD, sizeof(bufD));
        add(tgtCol, " direct dV %.2f km/s  burn %s", dvPCdirect, bufD);
    }
    sep();

    // Nodes
    add(kCyan,   "NODES  (cur -> tgt)");
    if (hasNodes) {
        char bufAN[12], bufDN[12];
        fmtHMS(tToAN, bufAN, sizeof(bufAN));
        fmtHMS(tToDN, bufDN, sizeof(bufDN));
        add(IM_COL32(100,255,100,230), " AN  TA %.1f  T+ %s", taDeg_AN, bufAN);
        add(IM_COL32(255,100,100,230), " DN  TA %.1f  T+ %s", taDeg_DN, bufDN);
    } else {
        add(kDim, " (coplanar)");
    }
    sep();

    // Burn + TMI
    add(kCyan,   "BURN  TA %.1f  Alt %.0f km", burnTADeg, altBurn);
    add(kDim,    "TMI  alt %.0f km  [ALT]", kParkAlts[m_parkIdx]);
    add(kYellow, " dV-TMI %.3f km/s  (Vp %.3f  Vc %.3f)", dvTMI, vPeri, vCirc);
    {
        char bufBurn[12], bufStart[12], bufToBurn[12];
        fmtHMS(burnDuration,    bufBurn,   sizeof(bufBurn));
        fmtHMS(timeToBurnPoint, bufToBurn, sizeof(bufToBurn));
        fmtHMS(std::max(0.0, timeToStartBurn), bufStart, sizeof(bufStart));
        add(kDim,    " accel %.3f m/s²  burn dur %s", accelMs2, bufBurn);
        add(kGreen,  " T-to-burn-point %s", bufToBurn);
        add(kYellow, " T-to-ignition   %s  (T-0.5burn)", bufStart);
    }
    sep();
    // Live ΔV remaining — updates in real time as the engine burns.
    {
        char bufRem[12];
        fmtHMS(burnRemaining, bufRem, sizeof(bufRem));
        ImU32 dvCol = (dvRemaining < 0.001) ? kGreen :
                      (dvRemaining < dvTMI * 0.1) ? kYellow : kOrange;
        add(dvCol, "dV-REMAINING %.3f km/s  (%s)", dvRemaining, bufRem);
        if (dvRemaining < 0.001)
            add(kGreen, " BURN COMPLETE");
    }

    // Measure max text width
    float maxW = 0.0f;
    for (auto& l : lines)
        if (l.col) maxW = std::max(maxW, ImGui::CalcTextSize(l.txt).x);

    float bx0 = origin.x + pad;
    float by0 = origin.y + pad;
    float bx1 = bx0 + maxW + pad * 2.0f;
    float by1 = by0 + static_cast<float>(lines.size()) * lineH + pad;

    dl->AddRectFilled({bx0, by0}, {bx1, by1}, kBacking, 3.0f);

    float tx = bx0 + pad, ty = by0 + pad * 0.5f;
    for (auto& l : lines) {
        if (l.col) dl->AddText({tx, ty}, l.col, l.txt);
        ty += lineH;
    }
}

// ---------------------------------------------------------------------------
// Page 3: heliocentric coasting view.
//
// Shows the ship's progress along the planned Lambert arc.  Uses the
// heliocentric ship state fed from main.cpp (Earth heliocentric + geocentric
// ship position).
//
// Re-solves Lambert from current heliocentric position to the planned Mars
// arrival point to compute the mid-course correction (MCC) ΔV.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// tickMcc — called every frame from main.cpp regardless of active view.
// Keeps m_mccDv current so the HUD marker and autopilot direction are always
// based on the latest ship trajectory.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// tickBplane — B-plane periapsis + inclination targeting for arrival body approach.
// Computes the minimum-norm ΔV that moves the B-vector from its current value
// (derived from the actual hyperbolic orbit) to the target (Pe + optional incl).
// Uses the exact lever-arm formula δv = −v∞·dB / (r·Ŝ) where Ŝ is the actual
// incoming asymptote direction computed from the eccentricity vector.
// ---------------------------------------------------------------------------
void TransferMFD::tickBplane()
{
    m_bplaneDv             = glm::dvec3(0.0);
    m_bplaneValid          = false;
    m_bplanePeCurrentKm    = 0.0;
    m_capturedAtArrival    = false;
    m_shipArrR             = glm::dvec3(0.0);
    m_shipArrV             = glm::dvec3(0.0);
    m_muArr                = 0.0;
    m_arrBodyRadius        = 0.0;
    m_insertionIncDeg      = 0.0;
    m_minAchievableIncDeg  = 0.0;

    if (!m_detail.valid) return;

    const int arrNaif = m_params.arrivalBody;   // e.g. 4 = Mars barycenter

    // Arrival-body μ and radius — try the exact planet ID first (arrNaif*100+99),
    // then fall back to the barycenter ID (arrNaif).
    double muArr = 0.0;
    double arrRadius = 0.0;
    {
        int planetId = (arrNaif < 10) ? arrNaif * 100 + 99 : arrNaif;
        try { astro::Spice().getPlanetaryConstants(planetId, "GM", muArr); } catch (...) {}
        if (muArr <= 0.0)
            try { astro::Spice().getPlanetaryConstants(arrNaif, "GM", muArr); } catch (...) {}
        if (muArr <= 0.0) return;   // no GM — can't compute

        astro::Vec3 radii;
        try { astro::Spice().getPlanetaryConstants(planetId, "RADII", radii);
              arrRadius = radii.x; } catch (...) {}
        if (arrRadius <= 0.0)
            try { astro::Vec3 r2;
                  astro::Spice().getPlanetaryConstants(arrNaif, "RADII", r2);
                  arrRadius = r2.x; } catch (...) {}
        if (arrRadius <= 0.0) return;
    }

    // Central body μ (Sun by default).
    double muCentral = m_params.muCentral;
    if (muCentral <= 0.0) {
        try { astro::Spice().getPlanetaryConstants(m_params.centralBody,
                                                   "GM", muCentral); } catch (...) {}
        if (muCentral <= 0.0) muCentral = 1.32712440018e11;
    }

    // Arrival-body heliocentric state.
    static const astro::ReferenceFrame kEclipJ2000bp =
        astro::ReferenceFrame::createEclipJ2000();
    astro::PosState arrState;
    try {
        astro::Spice().getRelativeGeometricState(arrNaif, m_params.centralBody,
            astro::EphemerisTime(m_currentET), arrState, kEclipJ2000bp);
    } catch (...) { return; }

    // Body-relative ship state — cache for tickApproach() and m_moiBurnCtrl.tick().
    glm::dvec3 r = m_shipHelioR - glm::dvec3(arrState.r.x, arrState.r.y, arrState.r.z);
    glm::dvec3 v = m_shipHelioV - glm::dvec3(arrState.v.x, arrState.v.y, arrState.v.z);
    m_shipArrR      = r;
    m_shipArrV      = v;
    m_muArr         = muArr;
    m_arrBodyRadius = arrRadius;

    double rMag = glm::length(r);
    if (rMag < 100.0) return;   // too close to body centre (inside body)

    // No upper distance threshold: the lever-arm formula ΔV = v∞×|Δb|/r is
    // valid at any range where v∞² > 0.  The ΔV naturally shrinks with
    // distance, so the display is useful throughout the approach.
    // (SOI is computed for informational use only, not as a gate.)

    // Hyperbolic excess speed v∞² = v² − 2μ/r.
    double vInfSq = glm::dot(v, v) - 2.0 * muArr / rMag;
    if (vInfSq <= 0.0) {
        // Captured in elliptic orbit — B-plane correction no longer applies,
        // but compute Pe altitude and actual inclination from orbital elements.
        const double v2  = glm::dot(v, v);
        const double rv  = glm::dot(r, v);
        const glm::dvec3 eVec = ((v2 - muArr / rMag) * r - rv * v) / muArr;
        const double ecc = glm::length(eVec);
        if (ecc >= 0.0 && ecc < 1.0) {
            const double energy = v2 * 0.5 - muArr / rMag;
            const double sma    = -muArr / (2.0 * energy);
            m_bplanePeCurrentKm = sma * (1.0 - ecc) - arrRadius;
            m_bplaneValid       = true;
            m_capturedAtArrival = true;
            // Compute actual orbit inclination from angular momentum vector.
            const glm::dvec3 hCapVec = glm::cross(r, v);
            const double     hCap    = glm::length(hCapVec);
            if (hCap > 1e-6) {
                const glm::dvec3 zEcl(0.0, 0.0, 1.0);
                m_insertionIncDeg = std::acos(std::clamp(
                    glm::dot(hCapVec / hCap, zEcl), -1.0, 1.0)) * 180.0 / M_PI;
            }
        }
        return;   // skip hyperbolic BPL dV computation
    }
    double vInf = std::sqrt(vInfSq);

    // Angular momentum vector, impact parameter b = h / v∞.
    glm::dvec3 hVec = glm::cross(r, v);
    double h = glm::length(hVec);
    if (h < 1e-6) return;
    double b = h / vInf;

    // Current periapsis radius: r_pe = −α + √(α² + b²), α = μ/v∞².
    double alpha = muArr / vInfSq;
    double rPe   = -alpha + std::sqrt(alpha * alpha + b * b);
    m_bplanePeCurrentKm = rPe - arrRadius;

    // Target impact parameter magnitude from desired Pe altitude.
    const double rPeTarget = arrRadius + kPeTargetAlts[m_peAltIdx];
    const double bTarget   = std::sqrt(rPeTarget * rPeTarget
                                       + 2.0 * muArr * rPeTarget / vInfSq);

    // ---- Current orbit normal and ecliptic inclination ----
    const glm::dvec3 hHat = hVec / h;
    const glm::dvec3 zEcl(0.0, 0.0, 1.0);
    m_insertionIncDeg = std::acos(std::clamp(glm::dot(hHat, zEcl), -1.0, 1.0))
                      * 180.0 / M_PI;

    // ---- Actual incoming asymptote direction from current orbit ----
    // Compute Ŝ from the eccentricity vector of the current hyperbola:
    //   Ŝ = ê/e + √(1 − 1/e²) · (ĥ × ê)    (unit vector, exact)
    // This avoids depending on the Lambert plan and works correctly regardless
    // of how far the current trajectory has drifted from the planned one.
    glm::dvec3 sHat;
    {
        const double vSq2 = glm::dot(v, v);
        const double rv2  = glm::dot(r, v);
        const glm::dvec3 eVec2 = ((vSq2 - muArr / rMag) * r - rv2 * v) / muArr;
        const double     ecc2  = glm::length(eVec2);
        if (ecc2 > 1.0 + 1e-6) {
            const glm::dvec3 eHat2 = eVec2 / ecc2;
            const glm::dvec3 qHat2 = glm::cross(hHat, eHat2);
            const double     sinA  = std::sqrt(std::max(0.0, 1.0 - 1.0 / (ecc2 * ecc2)));
            sHat = eHat2 / ecc2 + sinA * qHat2;   // already unit
        } else {
            sHat = glm::normalize(v);              // elliptic — use velocity direction
        }
    }
    glm::dvec3 tHat, rHatBpl;
    {
        glm::dvec3 tRaw = glm::cross(sHat, zEcl);
        if (glm::length(tRaw) < 1e-6) {
            tHat = glm::normalize(glm::cross(sHat, glm::dvec3(1.0, 0.0, 0.0)));
        } else {
            tHat = glm::normalize(tRaw);
        }
        rHatBpl = glm::cross(sHat, tHat);  // already unit (sHat ⊥ tHat)
    }

    // ---- Current B-vector phase angle in the B-plane ----
    // Derived from: orbit normal ĥ = -cos(φ)·R̂ + sin(φ)·T̂
    // So φ = atan2(ĥ·T̂, -(ĥ·R̂))
    const double phiCurrent = std::atan2(glm::dot(hHat, tHat),
                                         -glm::dot(hHat, rHatBpl));

    // ---- Target B-vector phase angle (from inclination preset) ----
    double phiTarget;
    if (m_incPresetIdx <= 0 || kIncPresetDeg[m_incPresetIdx] < 0.0) {
        // FREE mode: keep current B direction, only change |B|.
        phiTarget = phiCurrent;
    } else {
        // Solve: cos(inc) = (T̂·ẑ)·sin(φ) - (R̂·ẑ)·cos(φ) for φ.
        const double incTgtRad = kIncPresetDeg[m_incPresetIdx] * M_PI / 180.0;
        const double cosInc    = std::cos(incTgtRad);
        const double a         = glm::dot(tHat, zEcl);
        const double c         = glm::dot(rHatBpl, zEcl);
        const double amp       = std::sqrt(a * a + c * c);
        m_minAchievableIncDeg  = std::acos(std::clamp(amp, 0.0, 1.0)) * 180.0 / M_PI;
        if (amp < 1e-9) {
            phiTarget = phiCurrent;  // degenerate geometry — keep current
        } else {
            // clamp cosInc/amp to [-1,1]: targets the closest achievable inclination
            // when the requested value is outside the achievable range.
            const double arg  = std::clamp(cosInc / amp, -1.0, 1.0);
            const double base = std::atan2(c, a);
            auto wrapPi = [](double x) {
                while (x >  M_PI) x -= 2.0 * M_PI;
                while (x < -M_PI) x += 2.0 * M_PI;
                return x;
            };
            double phi0 = wrapPi(base + std::asin(arg));
            double phi1 = wrapPi(base + M_PI - std::asin(arg));
            // Pick the solution closest to current φ (minimises |ΔV|).
            double d0 = std::abs(wrapPi(phi0 - phiCurrent));
            double d1 = std::abs(wrapPi(phi1 - phiCurrent));
            phiTarget = (d0 <= d1) ? phi0 : phi1;
        }
    }

    // ---- Compute target B-vector and ΔB ----
    const double btTarget  = bTarget * std::cos(phiTarget);
    const double brTarget  = bTarget * std::sin(phiTarget);
    const double btCurrent = b * std::cos(phiCurrent);
    const double brCurrent = b * std::sin(phiCurrent);

    const glm::dvec3 dB = (btTarget - btCurrent) * tHat
                        + (brTarget - brCurrent) * rHatBpl;

    // Exact lever-arm: δB_actual = (r × δv) × Ŝ / v∞.
    // Min-norm solution (δv ⊥ Ŝ): δv = v∞ · δB_desired / (r·Ŝ).
    // B̂_actual = ĥ × Ŝ = −B̂_code, so δB_desired = −dB.
    // → δv = −v∞ · dB / (r·Ŝ).
    // r·Ŝ < 0 on the incoming leg (r anti-parallel to Ŝ), so δv is in the +dB direction.
    const double rDotS = glm::dot(r, sHat);
    if (rDotS >= 0.0) {
        // Ship is past periapsis on the outbound leg — no further B-plane correction.
        m_bplaneValid = true;
        return;
    }
    m_bplaneDv    = -(vInf / rDotS) * dB;   // rDotS < 0, so scalar is positive
    m_bplaneValid = true;
}

bool TransferMFD::inArrivalSoi() const
{
    if (m_muArr <= 0.0) return false;
    // Distance from ship to arrival body (arrival-body-centric position).
    const double distKm = glm::length(m_shipArrR);
    if (distKm < 1.0) return false;   // not yet initialised

    // SOI = a_body * (μ_body / μ_sun)^(2/5); approximate a_body as the
    // heliocentric distance of the arrival body = |shipHelio − shipArrR|.
    const double muSun  = (m_params.muCentral > 0.0) ? m_params.muCentral : 1.32712440018e11;
    const double aBody  = glm::length(m_shipHelioR - m_shipArrR);
    if (aBody < 1.0) return false;
    const double soiKm  = aBody * std::pow(m_muArr / muSun, 0.4);
    return distKm < soiKm;
}

// ---------------------------------------------------------------------------
// tickApproach — computes time-to-Pe and MOI burn parameters from cached
// arrival-body state.  Called from update() after tickBplane().
// Results cached in m_tToPeHyp / m_moiIgnET / m_moiDvCirc / m_moiBurnDur.
// ---------------------------------------------------------------------------
void TransferMFD::tickApproach()
{
    m_tToPeHyp   = std::numeric_limits<double>::quiet_NaN();
    m_moiIgnET   = 0.0;
    m_moiDvCirc  = 0.0;
    m_moiBurnDur = 0.0;

    if (!m_bplaneValid || m_muArr <= 0.0 || m_arrBodyRadius <= 0.0) return;

    const double rMag = glm::length(m_shipArrR);
    if (rMag < 100.0) return;

    const double v2     = glm::dot(m_shipArrV, m_shipArrV);
    const double vInfSq = v2 - 2.0 * m_muArr / rMag;
    const double rv     = glm::dot(m_shipArrR, m_shipArrV);
    const glm::dvec3 eVec = ((v2 - m_muArr / rMag) * m_shipArrR - rv * m_shipArrV) / m_muArr;
    const double ecc = glm::length(eVec);

    if (vInfSq > 0.0 && ecc > 1.0) {
        // Hyperbolic approach: time to periapsis via hyperbolic anomaly.
        const double energy = 0.5 * v2 - m_muArr / rMag;
        const double absA   = m_muArr / (2.0 * std::abs(energy));
        const double cosNu  = std::clamp(glm::dot(eVec / ecc, m_shipArrR / rMag), -1.0, 1.0);
        double nu = std::acos(cosNu);
        if (rv < 0.0) nu = -nu;
        const double k    = std::sqrt((ecc - 1.0) / (ecc + 1.0));
        const double argF = k * std::tan(nu * 0.5);
        if (std::isfinite(argF) && std::abs(argF) < 1.0 - 1e-9) {
            const double F = 2.0 * std::atanh(argF);
            if (std::isfinite(F) && absA > 1.0) {
                const double M_h = ecc * std::sinh(F) - F;
                const double n   = std::sqrt(m_muArr / (absA * absA * absA));
                m_tToPeHyp = -(M_h / n);
            }
        }
    } else if (m_capturedAtArrival && ecc > 0.0 && ecc < 1.0) {
        // Captured in elliptic orbit: time to next periapsis via Kepler.
        const double energy = 0.5 * v2 - m_muArr / rMag;
        const double sma    = -m_muArr / (2.0 * energy);
        if (sma > 0.0) {
            const double cosNu = std::clamp(glm::dot(eVec / ecc, m_shipArrR / rMag), -1.0, 1.0);
            double nu = std::acos(cosNu);
            if (rv < 0.0) nu = -nu;
            const double tanH    = std::tan(nu * 0.5) * std::sqrt((1.0 - ecc) / (1.0 + ecc));
            const double E       = 2.0 * std::atan(tanH);
            const double M       = E - ecc * std::sin(E);
            const double n       = std::sqrt(m_muArr / (sma * sma * sma));
            const double T       = 2.0 * M_PI / n;
            const double tFromPe = M / n;
            m_tToPeHyp = (tFromPe >= 0.0) ? -(T - tFromPe) : -tFromPe;

            // MOI burn parameters for elliptic circularization at next Pe.
            if (std::isfinite(m_tToPeHyp)) {
                const double rPeCurrent = sma * (1.0 - ecc);
                const double vAtPe      = std::sqrt(m_muArr * (2.0 / rPeCurrent - 1.0 / sma));
                const double rPeTgt     = m_arrBodyRadius + kPeTargetAlts[m_peAltIdx];
                const double vCircAtPe  = std::sqrt(m_muArr / rPeTgt);
                m_moiDvCirc  = std::max(0.0, vAtPe - vCircAtPe);
                const double accel = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
                m_moiBurnDur = m_moiDvCirc * 1000.0 / accel;
                const double tIgn = m_tToPeHyp - m_moiBurnDur * 0.5;
                m_moiIgnET = (tIgn > 0.0) ? m_currentET + tIgn : 0.0;

                // Pred Pe is NOT recomputed here — it keeps the last value
                // from the hyperbolic approach, which was accurate and stable.
            }
        }
    }

    // MOI burn parameters — hyperbolic approach only.
    if (std::isfinite(m_tToPeHyp) && !m_capturedAtArrival && vInfSq > 0.0) {
        const double rPeTgt    = m_arrBodyRadius + kPeTargetAlts[m_peAltIdx];
        const double vAtPe     = std::sqrt(vInfSq + 2.0 * m_muArr / rPeTgt);
        const double vCircAtPe = std::sqrt(m_muArr / rPeTgt);
        m_moiDvCirc  = std::max(0.0, vAtPe - vCircAtPe);
        const double accel = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
        m_moiBurnDur = m_moiDvCirc * 1000.0 / accel;
        const double tIgn = m_tToPeHyp - m_moiBurnDur * 0.5;
        m_moiIgnET = (tIgn > 0.0) ? m_currentET + tIgn : 0.0;

        // Predict the finite-burn Pe: analytically propagate the hyperbolic
        // conic to the ignition point (T−0.5burn), then RK4 through the burn.
        m_predictedPeKm = std::numeric_limits<double>::quiet_NaN();
        if (m_moiBurnDur > 0.0 && ecc > 1.0) {
            const double mu  = m_muArr;
            const glm::dvec3 hVec  = glm::cross(m_shipArrR, m_shipArrV);
            const double hMag      = glm::length(hVec);
            const double absA      = mu / (2.0 * std::abs(0.5*v2 - mu/rMag));
            const double n         = std::sqrt(mu / (absA * absA * absA));
            const double p         = absA * (ecc * ecc - 1.0);

            const double Mh = -n * (m_moiBurnDur * 0.5);
            double F = Mh;
            for (int i = 0; i < 50; ++i) {
                const double dF = -(ecc * std::sinh(F) - F - Mh)
                                 / (ecc * std::cosh(F) - 1.0);
                F += dF;
                if (std::abs(dF) < 1e-12) break;
            }
            const double k   = std::sqrt((ecc - 1.0) / (ecc + 1.0));
            const double nu  = 2.0 * std::atan(std::tanh(F * 0.5) / k);
            const double cosNu = std::cos(nu), sinNu = std::sin(nu);

            const glm::dvec3 periDir = eVec / ecc;
            const glm::dvec3 qDir    = glm::cross(hVec / hMag, periDir);
            const double rIgnMag     = p / (1.0 + ecc * cosNu);
            const glm::dvec3 rIgn    = rIgnMag * (cosNu * periDir + sinNu * qDir);
            const double sqMuP       = std::sqrt(mu / p);
            const glm::dvec3 vIgn    = sqMuP * (-sinNu * periDir + (ecc + cosNu) * qDir);

            const double accelKmS2 = accel / 1000.0;
            auto [rf, vf] = spacecraft::propagateFiniteBurn(
                rIgn, vIgn, mu, accelKmS2,
                0.0, 0, m_moiBurnDur, 200);
            m_predictedPeKm = spacecraft::periapsisAltitude(rf, vf, mu, m_arrBodyRadius);
        }
    }
}

void TransferMFD::tickMcc()
{
    m_mccDv = glm::dvec3(0.0);
    if (!m_detail.valid) return;

    // The MCC correction is computed with a heliocentric two-body model.
    // Inside Earth's sphere of influence (~929,000 km) the geocentric
    // velocity dominates and the heliocentric velocity vector looks nothing
    // like the asymptotic departure velocity — the Lambert result would be
    // completely wrong (17+ km/s artifacts).  Suppress until clear of SOI.
    if (glm::length(m_shipR) < m_depSOI) return;

    // Inside the arrival SOI the ship's heliocentric position is essentially
    // at the target, so Lambert degenerates and produces garbage (50+ km/s).
    // B-plane targeting handles corrections from this point on.
    if (inArrivalSoi()) return;

    const double tofRemaining = m_detail.arrET - m_currentET;
    if (tofRemaining <= kDay) return;

    double muSun = m_params.muCentral;
    if (muSun <= 0.0) {
        try { astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", muSun); }
        catch (...) {}
    }
    if (muSun <= 0.0) muSun = 1.32712440018e11;

    glm::dvec3 vMCC_dep, vMCC_arr;
    if (spacecraft::solveLambert(muSun, m_shipHelioR, m_detail.arrPos,
                                 tofRemaining, true, vMCC_dep, vMCC_arr)) {
        m_mccDv = vMCC_dep - m_shipHelioV;
    }

    // MCC deviation warning: fire voice alert on rising edge, drop sim speed once.
    // Use the same active-nav-dV selection as the HUD diamond and burn AP:
    // B-plane correction when available (late approach or Pe < 50,000 km), else Lambert MCC.
    if (m_mccWarnIdx > 0) {
        const bool   bplUsable = m_bplaneValid
                              && (glm::length(m_bplaneDv) > 1e-9)
                              && (m_bplanePeCurrentKm < 50000.0 || getTransferProgress() >= 0.97);
        const double dv        = bplUsable ? glm::length(m_bplaneDv)
                                           : glm::length(m_mccDv);
        const double thresh = kMccWarnKms[m_mccWarnIdx];
        if (dv > thresh) {
            if (!m_mccWarnActive) {
                char buf[80];
                std::snprintf(buf, sizeof(buf),
                    "Course correction warning. %.0f meters per second.",
                    dv * 1000.0);
                obc::speakImmediate(buf);
            }
            if (!m_mccWarnActive)
                m_mccWarnDropPending = true;  // rising edge — drop to 1× once
            m_mccWarnActive = true;
        } else {
            m_mccWarnActive = false;
        }
    } else {
        m_mccWarnActive = false;
    }
}

void TransferMFD::renderCoasting(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen   = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim     = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow  = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan    = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange  = IM_COL32(255, 140,   0, 220);
    const ImU32 kBacking = IM_COL32(  0,   0,   0, 175);

    if (!m_detail.valid) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim, "No transfer selected");
        return;
    }

    // -----------------------------------------------------------------------
    // Sun GM
    // -----------------------------------------------------------------------
    double muSun = m_params.muCentral;
    if (muSun <= 0.0) {
        try { astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", muSun); }
        catch (...) {}
    }
    if (muSun <= 0.0) muSun = 1.32712440018e11;  // fallback km³/s²

    // -----------------------------------------------------------------------
    // Heliocentric ship state — fed by main.cpp via updateHelioState().
    // -----------------------------------------------------------------------
    const glm::dvec3 shipR = m_shipHelioR;
    const glm::dvec3 shipV = m_shipHelioV;

    // -----------------------------------------------------------------------
    // Current positions of departure and arrival bodies from SPICE
    // -----------------------------------------------------------------------
    const astro::EphemerisTime etNow(m_currentET);
    astro::PosState depNow, arrNow;
    try {
        astro::Spice().getRelativeGeometricState(
            m_params.departureBody, m_params.centralBody, etNow, depNow, kEclipJ2000);
        astro::Spice().getRelativeGeometricState(
            m_params.arrivalBody,   m_params.centralBody, etNow, arrNow, kEclipJ2000);
    } catch (...) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim,
                    "SPICE query failed");
        return;
    }

    // -----------------------------------------------------------------------
    // Lambert arc builder helper (position r1, velocity v1, endpoint r2)
    // -----------------------------------------------------------------------
    auto buildArc = [&](const glm::dvec3& r1, const glm::dvec3& v1,
                        const glm::dvec3& r2, double mu)
            -> std::vector<glm::dvec3>
    {
        std::vector<glm::dvec3> pts;
        glm::dvec3 h    = glm::cross(r1, v1);
        double     hMag = glm::length(h);
        if (hMag < 1e-6) return pts;
        glm::dvec3 hn      = h / hMag;
        double     p_      = hMag * hMag / mu;
        glm::dvec3 ev      = glm::cross(v1, h) / mu - glm::normalize(r1);
        double     ecc     = glm::length(ev);
        glm::dvec3 periDir = (ecc > 1e-9) ? glm::normalize(ev)
                                           : glm::normalize(r1);
        glm::dvec3 qDir    = glm::cross(hn, periDir);

        auto nu_of = [&](const glm::dvec3& r) -> double {
            double cosnu = glm::dot(periDir, glm::normalize(r));
            cosnu = std::clamp(cosnu, -1.0, 1.0);
            double nu = std::acos(cosnu);
            if (glm::dot(glm::cross(periDir, r), hn) < 0.0) nu = -nu;
            return nu;
        };

        double nu1 = nu_of(r1);
        double nu2 = nu_of(r2);
        if (nu2 < nu1) nu2 += 2.0 * M_PI;

        for (int k = 0; k <= 80; ++k) {
            double nu = nu1 + (nu2 - nu1) * k / 80.0;
            double r_ = p_ / (1.0 + ecc * std::cos(nu));
            pts.push_back(r_ * (std::cos(nu) * periDir + std::sin(nu) * qDir));
        }
        return pts;
    };

    // -----------------------------------------------------------------------
    // Planned Lambert arc (departure → arrival)
    // -----------------------------------------------------------------------
    OrbitDiagram::Arc planArc;
    planArc.colour    = IM_COL32(255, 220, 0, 120);   // dim yellow: the plan
    planArc.thickness = 1.2f;
    planArc.pts = buildArc(m_detail.depPos, m_detail.vDep,
                           m_detail.arrPos, muSun);

    // -----------------------------------------------------------------------
    // Mid-course correction (MCC): re-solve Lambert from now to arrival.
    // Only meaningful when inside the transfer window.
    // -----------------------------------------------------------------------
    double tofRemaining = m_detail.arrET - m_currentET;
    glm::dvec3 vMCC_dep, vMCC_arr;
    bool   mccValid = false;
    double dvMCC    = 0.0;
    OrbitDiagram::Arc mccArc;

    const bool insideDepSOI = (glm::length(m_shipR) < m_depSOI);

    if (!insideDepSOI && tofRemaining > kDay) {   // at least 1 day remaining
        mccValid = spacecraft::solveLambert(muSun, shipR, m_detail.arrPos,
                                            tofRemaining, true,
                                            vMCC_dep, vMCC_arr);
        if (mccValid) {
            dvMCC   = glm::length(vMCC_dep - shipV);
            m_mccDv = vMCC_dep - shipV;   // heliocentric ΔV vector, km/s
            // Draw MCC arc only when noticeably off-nominal (> 0.5 m/s).
            if (dvMCC > 5e-4) {
                mccArc.colour    = kCyan;
                mccArc.thickness = 1.5f;
                mccArc.pts = buildArc(shipR, vMCC_dep, m_detail.arrPos, muSun);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Transfer progress
    // -----------------------------------------------------------------------
    double elapsed  = m_currentET - m_detail.depET;
    double tofTotal = m_detail.tofSec;
    double progress = (tofTotal > 0.0)
                    ? std::clamp(elapsed / tofTotal, 0.0, 1.0) : 0.0;
    int elapsedDays  = static_cast<int>(elapsed  / kDay);
    int remainDays   = static_cast<int>(tofRemaining / kDay);

    double helioSpeed  = glm::length(shipV);
    double distToArr   = glm::length(arrNow.r - shipR);  // km to arrival body
    double distToDep   = glm::length(depNow.r - shipR);  // km to departure body

    // -----------------------------------------------------------------------
    // Orbit diagram (heliocentric, Sun at centre)
    // -----------------------------------------------------------------------
    OrbitDiagram diag;

    diag.addOrbit(depNow.r, depNow.v, muSun,
                  IM_COL32(60, 140, 255, 80), "", false, false);
    diag.addOrbit(arrNow.r, arrNow.v, muSun,
                  IM_COL32(200, 80, 50, 80), "", false, false);

    // Planned Lambert arc
    diag.addArc(planArc);

    // MCC arc (if off-nominal)
    if (!mccArc.pts.empty())
        diag.addArc(mccArc);

    // Departure and arrival reference points
    diag.addMarker(m_detail.depPos, IM_COL32(60, 140, 255, 150), "D");
    diag.addMarker(m_detail.arrPos, IM_COL32(200, 80, 50, 150), "A");

    diag.addMarker(depNow.r, IM_COL32(60, 140, 255, 255), m_depBodyName.substr(0,1).c_str());
    diag.addMarker(arrNow.r, IM_COL32(200, 80, 50, 255),  m_arrBodyName.substr(0,1).c_str());
    diag.addMarker(shipR,    kGreen, "S");

    // Sun
    OrbitDiagram::CentralBody sun;
    sun.rimColour  = IM_COL32(255, 210, 60, 200);
    sun.axisColour = IM_COL32(255, 210, 60, 80);
    sun.drawAxes   = false;
    diag.setCentralBody(sun);

    diag.render(dl, origin, size, &m_coastViewRot);

    // tToPeHyp is kept current by tickApproach() (called from update() each frame).
    const double tToPeHyp = m_tToPeHyp;

    // -----------------------------------------------------------------------
    // Text overlay
    // -----------------------------------------------------------------------
    const float pad   = 3.0f;
    const float lineH = 11.0f;

    struct Line { ImU32 col; char txt[80]; };
    std::vector<Line> lines;
    auto addLine = [&](ImU32 col, const char* fmt, ...) {
        Line l; l.col = col;
        va_list ap; va_start(ap, fmt);
        std::vsnprintf(l.txt, sizeof(l.txt), fmt, ap);
        va_end(ap);
        lines.push_back(l);
    };
    auto sep = [&]() {
        Line l; l.col = 0; l.txt[0] = '\0';
        lines.push_back(l);
    };

    // -- Dates ---------------------------------------------------------------
    {
        std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
        std::string arrStr = astro::EphemerisTime(m_detail.arrET).toISOUTCString(0);
        addLine(kDim,    "DEP  %.10s", depStr.c_str());
        addLine(kDim,    "ARR  %.10s", arrStr.c_str());
    }
    sep();

    // -- Progress bar --------------------------------------------------------
    {
        // ASCII progress bar: [=====>    ]  XX%
        const int barW = 18;
        int filled = static_cast<int>(progress * barW);
        char bar[24]; bar[0] = '[';
        for (int i = 0; i < barW; ++i)
            bar[1 + i] = (i < filled) ? '=' : (i == filled ? '>' : ' ');
        bar[1 + barW] = ']'; bar[2 + barW] = '\0';
        ImU32 pCol = (progress < 0.5) ? kGreen :
                     (progress < 0.9) ? kYellow : kOrange;
        addLine(pCol, "PROG %s %3.0f%%", bar, progress * 100.0);
        if (std::isfinite(tToPeHyp)) {
            // On hyperbolic approach: show live T-to-Pe instead of Lambert countdown.
            const bool neg = tToPeHyp < 0.0;
            int t = static_cast<int>(std::abs(tToPeHyp) + 0.5);
            const int hh = t / 3600; t %= 3600;
            const int mm = t / 60;   t %= 60;
            char bufPe[20];
            std::snprintf(bufPe, sizeof(bufPe), "%s%d:%02d:%02d",
                          neg ? "-" : "", hh, mm, t);
            const ImU32 peCol = neg ? kOrange : kCyan;
            addLine(peCol, " T+%d d  (T-to-Pe %s)", elapsedDays, bufPe);
        } else {
            addLine(kGreen, " T+%d d  (T-%d d to arrival)", elapsedDays, remainDays);
        }
    }
    sep();

    // -- Heliocentric state --------------------------------------------------
    {
        constexpr double kAU = 149597870.7;  // km per AU
        double rSun = glm::length(shipR);    // km from Sun
        addLine(kCyan,  "HELIO  v %.3f km/s  r %.3f AU",
                        helioSpeed, rSun / kAU);
        addLine(kDim,   " d%s %.1f Mkm  d%s %.1f Mkm",
                        m_depBodyName.substr(0,1).c_str(), distToDep / 1.0e6,
                        m_arrBodyName.substr(0,1).c_str(), distToArr / 1.0e6);
    }
    sep();

    // -- MCC ----------------------------------------------------------------
    if (!mccValid || tofRemaining <= kDay) {
        if (tofRemaining <= 0.0)
            addLine(kGreen,  "ARRIVAL  dV-arr %.3f km/s", m_detail.dv2);
        else
            addLine(kDim,    "MCC  N/A (near arrival)");
    } else {
        ImU32 mccCol = (dvMCC < 0.001) ? kGreen :
                       (dvMCC < 0.05)  ? kYellow : kOrange;
        addLine(mccCol,  "MCC  dV %.3f km/s", dvMCC);
        if (dvMCC < 0.001)
            addLine(kGreen,  " ON NOMINAL TRAJECTORY");
        else
            addLine(kDim,    " arr dV %.3f km/s (new)", glm::length(vMCC_arr - m_detail.vArrBody));
    }
    {
        static constexpr const char* kWarnLabels[] = {"OFF", "100 m/s", "200 m/s", "500 m/s"};
        if (m_mccWarnActive)
            addLine(kOrange, " WARN >%s  [ACTIVE]", kWarnLabels[m_mccWarnIdx]);
        else if (m_mccWarnIdx > 0)
            addLine(kDim,    " WARN >%s", kWarnLabels[m_mccWarnIdx]);
        else
            addLine(kDim,    " WARN  OFF");
    }

    // -- Approach status (brief) — full detail on ARR page ------------------
    if (m_bplaneValid) {
        sep();
        const double peNow = m_bplanePeCurrentKm;
        ImU32 peCol = (peNow > 0.0) ? kGreen : IM_COL32(255, 60, 60, 220);
        if (m_capturedAtArrival)
            addLine(kGreen, "CAPTURED  Pe %.0f km  [ARR]", peNow);
        else if (peNow > 0.0)
            addLine(peCol, "Pe now  %.0f km  [ARR for details]", peNow);
        else
            addLine(peCol, "Pe now  %.0f km  IMPACT  [ARR]", peNow);

        if (std::isfinite(tToPeHyp)) {
            const bool neg = tToPeHyp < 0.0;
            int t = static_cast<int>(std::abs(tToPeHyp) + 0.5);
            const int hh = t / 3600; t %= 3600;
            const int mm = t / 60;   t %= 60;
            char bufPe[20];
            std::snprintf(bufPe, sizeof(bufPe), "%s%d:%02d:%02d",
                          neg ? "-" : "", hh, mm, t);
            addLine((tToPeHyp > 0.0) ? kCyan : kOrange,
                    " T-to-Pe  %s", bufPe);
        }

        const auto moiPh = m_moiBurnCtrl.phase();
        if (moiPh == BurnPhase::Armed || moiPh == BurnPhase::PreIgnition) {
            addLine(kCyan, " MOI ARMED");
        } else if (moiPh == BurnPhase::Executing) {
            addLine(kOrange, " MOI EXECUTING");
        } else if (moiPh == BurnPhase::Complete) {
            addLine(kGreen,  " MOI COMPLETE");
        }
    }

    // -- Measure max width and draw backing ----------------------------------
    float maxW = 0.0f;
    for (auto& l : lines)
        if (l.col) maxW = std::max(maxW, ImGui::CalcTextSize(l.txt).x);

    float bx0 = origin.x + pad;
    float by0 = origin.y + pad;
    float bx1 = bx0 + maxW + pad * 2.0f;
    float by1 = by0 + static_cast<float>(lines.size()) * lineH + pad;

    dl->AddRectFilled({ bx0, by0 }, { bx1, by1 }, kBacking, 3.0f);

    float ox = bx0 + pad, oy = by0 + pad * 0.5f;
    for (auto& l : lines) {
        if (l.col) dl->AddText({ ox, oy }, l.col, l.txt);
        oy += lineH;
    }
}

// ---------------------------------------------------------------------------
// Page 5: arrival B-plane targeting.
//
// Shows the full B-vector correction (Pe + inclination), MOI burn planning,
// and the approach hyperbola from the arrival-body-centric perspective.
// ---------------------------------------------------------------------------
void TransferMFD::renderArrival(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen   = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim     = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow  = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan    = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange  = IM_COL32(255, 140,   0, 220);
    const ImU32 kRed     = IM_COL32(255,  60,  60, 220);
    const ImU32 kBacking = IM_COL32(  0,   0,   0, 175);

    if (!m_detail.valid) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim,
                    "No transfer selected");
        return;
    }

    // ---- Formatters --------------------------------------------------------
    auto fmtSgn = [](double s, char* b, int sz) {
        if (!std::isfinite(s)) { std::snprintf(b, sz, "--:--:--"); return; }
        const bool neg = s < 0.0;
        int t = static_cast<int>(std::abs(s) + 0.5);
        const int hh = t / 3600; t %= 3600;
        const int mm = t / 60;   t %= 60;
        std::snprintf(b, sz, "%s%d:%02d:%02d", neg ? "-" : "", hh, mm, t);
    };

    // ---- Lines buffer ------------------------------------------------------
    struct Line { ImU32 col; char txt[96]; };
    std::vector<Line> lines;
    auto add = [&](ImU32 col, const char* fmt, ...) {
        Line l; l.col = col;
        va_list ap; va_start(ap, fmt);
        std::vsnprintf(l.txt, sizeof(l.txt), fmt, ap);
        va_end(ap);
        lines.push_back(l);
    };
    auto sep = [&]() { Line l; l.col = 0; l.txt[0] = '\0'; lines.push_back(l); };

    // ---- Arrival v∞ --------------------------------------------------------
    {
        glm::dvec3 vInfVec = m_detail.vArr - m_detail.vArrBody;
        double     vInfMag = glm::length(vInfVec);
        add(kDim, "ARR  %s  ->  %s",
            m_depBodyName.c_str(), m_arrBodyName.c_str());
        add(kCyan, "v-inf  %.3f km/s", vInfMag);

        // v∞ inclination above ecliptic
        if (vInfMag > 1e-6) {
            double sinInc = vInfVec.z / vInfMag;
            double incDeg = std::asin(std::clamp(sinInc, -1.0, 1.0)) * 180.0 / M_PI;
            add(kDim, "  v-inf inc  %.1f° (ecl)", incDeg);
        }
    }
    sep();

    // ---- Current approach state --------------------------------------------
    if (!m_bplaneValid) {
        add(kDim, "B-plane not active");
        add(kDim, " (outside SOI or no v-inf)");
    } else {
        const double peNow  = m_bplanePeCurrentKm;
        const double peTgt  = kPeTargetAlts[m_peAltIdx];
        const double bplDv  = glm::length(m_bplaneDv);
        const double incNow = m_insertionIncDeg;

        // Pe current
        ImU32 peNowCol = (peNow > 0.0) ? kGreen : kRed;
        if (m_capturedAtArrival)
            add(kGreen, "Pe now  %.0f km  (captured)", peNow);
        else if (peNow > 0.0)
            add(peNowCol, "Pe now  %.0f km", peNow);
        else
            add(peNowCol, "Pe now  %.0f km  IMPACT", peNow);

        // Pe target
        add(kYellow, "Pe tgt  %.0f km  [PE]", peTgt);

        sep();

        // Insertion orbit inclination
        add(kCyan,   "Inc now  %.1f°  (ecl)", incNow);

        // Target inclination
        const char* presetName = (m_incPresetIdx >= 0 && m_incPresetIdx < kNIncPresets)
                               ? kIncPresetName[m_incPresetIdx] : "?";
        if (m_incPresetIdx == 0) {
            add(kDim, "Inc tgt  FREE (Pe only)  [INC]");
        } else {
            double tgtDeg = kIncPresetDeg[m_incPresetIdx];
            add(kYellow, "Inc tgt  %s = %.0f°  [INC]", presetName, tgtDeg);
            // Warn when the requested inclination is below the minimum achievable.
            if (!m_capturedAtArrival && m_minAchievableIncDeg > 0.1
                    && tgtDeg < m_minAchievableIncDeg - 0.5) {
                add(kOrange, "  Min achievable  %.1f°", m_minAchievableIncDeg);
            }
        }

        sep();

        // B-plane ΔV
        if (!m_capturedAtArrival) {
            ImU32 bplCol = (bplDv < 0.001) ? kGreen :
                           (bplDv < 0.1)   ? kYellow : kOrange;
            add(bplCol, "BPL dV  %.3f km/s", bplDv);
            if (bplDv < 0.001) {
                add(kGreen, " ON TARGET");
            } else {
                add(kDim,  "  %.4f  %.4f  %.4f  km/s",
                    m_bplaneDv.x, m_bplaneDv.y, m_bplaneDv.z);
            }
        } else {
            add(kGreen, "CAPTURED  (elliptic orbit)");
        }

        // ---- Time-to-Pe and MOI burn ----------------------------------------
        if (std::isfinite(m_tToPeHyp)) {
            sep();
            char bufTpe[16], bufBurn[16], bufIgn[16];
            const double tIgnRel = (m_moiIgnET > 0.0)
                                 ? m_moiIgnET - m_currentET
                                 : std::numeric_limits<double>::quiet_NaN();
            fmtSgn(m_tToPeHyp,   bufTpe,  sizeof(bufTpe));
            fmtSgn(m_moiBurnDur,  bufBurn, sizeof(bufBurn));
            fmtSgn(tIgnRel,       bufIgn,  sizeof(bufIgn));

            const ImU32 tPeCol = (m_tToPeHyp > 0.0) ? kCyan : kOrange;
            add(tPeCol,  "T-to-Pe   %s", bufTpe);

            if (m_moiDvCirc > 0.0) {
                add(kYellow, "MOI dV    %.3f km/s  (%s)", m_moiDvCirc, bufBurn);
                add(kCyan,   "T-ign     %s  (T-0.5burn)", bufIgn);
                if (std::isfinite(m_predictedPeKm)) {
                    const ImU32 peCol = (m_predictedPeKm < 0.0) ? kOrange : kYellow;
                    add(peCol, "Pred Pe   %.0f km  (finite burn)", m_predictedPeKm);
                }

                sep();

                // ARM status / button prompt
                const auto moiPh = m_moiBurnCtrl.phase();
                if (moiPh == BurnPhase::Armed || moiPh == BurnPhase::PreIgnition) {
                    char bufCd[16];
                    fmtSgn(m_moiBurnCtrl.plan().ignitionET - m_currentET,
                           bufCd, sizeof(bufCd));
                    add(kCyan,   "MOI ARMED  T-ign %s", bufCd);
                    add(kDim,    " [DSARM] to cancel");
                } else if (moiPh == BurnPhase::Executing) {
                    add(kOrange, "MOI EXECUTING");
                } else if (moiPh == BurnPhase::Complete) {
                    add(kGreen,  "MOI COMPLETE");
                } else if (m_moiIgnET > m_currentET) {
                    add(kDim,    "[ARM] to arm MOI autopilot");
                }
            }
        }
    }

    // ---- Draw text panel ---------------------------------------------------
    const float pad   = 3.0f;
    const float lineH = 11.0f;

    float maxW = 0.0f;
    for (auto& l : lines)
        if (l.col) maxW = std::max(maxW, ImGui::CalcTextSize(l.txt).x);

    float bx0 = origin.x + pad;
    float by0 = origin.y + pad;
    float bx1 = bx0 + maxW + pad * 2.0f;
    float by1 = by0 + static_cast<float>(lines.size()) * lineH + pad;

    dl->AddRectFilled({ bx0, by0 }, { bx1, by1 }, kBacking, 3.0f);

    float tx = bx0 + pad, ty = by0 + pad * 0.5f;
    for (auto& l : lines) {
        if (l.col) dl->AddText({ tx, ty }, l.col, l.txt);
        ty += lineH;
    }

    // ---- Orbit diagram: arrival body centric approach ----------------------
    if (m_bplaneValid && m_muArr > 0.0 && m_arrBodyRadius > 0.0) {
        const float diagY0  = by1 + pad * 2.0f;
        const float diagH   = origin.y + size.y - diagY0 - pad;
        if (diagH >= 40.0f) {
            OrbitDiagram diag;

            // Arrival body as central body
            OrbitDiagram::CentralBody cb;
            cb.radiusKm   = m_arrBodyRadius;
            cb.rimColour  = IM_COL32(200, 80, 50, 200);
            cb.axisColour = IM_COL32(200, 80, 50, 60);
            cb.drawAxes   = false;
            diag.setCentralBody(cb);

            // Ship approach trajectory (body-centric state)
            if (glm::length(m_shipArrR) > 100.0) {
                OrbitDiagram::Orbit o;
                o.r           = m_shipArrR;
                o.v           = m_shipArrV;
                o.mu          = m_muArr;
                o.colour      = kCyan;
                o.showApses   = true;
                o.showCurrent = true;
                diag.addOrbit(o);
            }

            diag.render(dl, { origin.x, diagY0 }, { size.x, diagH },
                        &m_arrViewRot);
        }
    }
}

// ---------------------------------------------------------------------------
void TransferMFD::update(const MFDContext& ctx)
{
    m_eventQueue = ctx.eventQueue;
    m_autopilot  = ctx.autopilot;

    // Populate body list from context on first call (or if it changes).
    if (m_bodies.size() != ctx.solarSystemBodies.size()) {
        m_bodies = ctx.solarSystemBodies;
        // Resolve indices from the current NAIF IDs (may have been set by setEpoch
        // or restorePlan before the body list was available).
        int di = findPlanetIdx(m_params.departureBody);
        int ai = findPlanetIdx(m_params.arrivalBody);
        if (di < 0) di = findPlanetIdx(399);   // fallback: Earth
        if (ai < 0) ai = findPlanetIdx(499);   // fallback: Mars
        // Also try barycenters if planet-centre IDs aren't in the list.
        if (di < 0) di = findPlanetIdx(3);
        if (ai < 0) ai = findPlanetIdx(4);
        if (di >= 0) { m_depPlanetIdx = di; m_depBodyName = m_bodies[di].shortName;
                       m_params.departureBody = m_bodies[di].naifId; queryDepBodyConstants(); }
        if (ai >= 0) { m_arrPlanetIdx = ai; m_arrBodyName = m_bodies[ai].shortName;
                       m_params.arrivalBody   = m_bodies[ai].naifId; }
        updateDefaultTof();
    }

    // Compute departure-body-centric state from heliocentric.
    // For Earth (399) the dedicated geocentric field is more precise;
    // for any other departure body subtract the body's heliocentric position.
    glm::dvec3 shipR, shipV;
    if (m_params.departureBody == 399) {
        shipR = ctx.shipGeoR;
        shipV = ctx.shipGeoV;
    } else {
        try {
            astro::PosState depState;
            astro::Spice().getRelativeGeometricState(
                m_params.departureBody, m_params.centralBody,
                ctx.currentEt, depState, kEclipJ2000);
            glm::dvec3 rDep(depState.r.x, depState.r.y, depState.r.z);
            glm::dvec3 vDep(depState.v.x, depState.v.y, depState.v.z);
            shipR = ctx.shipHelioR - rDep;
            shipV = ctx.shipHelioV - vDep;
        } catch (...) {
            shipR = ctx.shipGeoR;
            shipV = ctx.shipGeoV;
        }
    }

    updateShipState(shipR, shipV, ctx.currentEt.getETValue());
    updateBurnParams(ctx.mainThrustN, ctx.shipMassKg);
    updateHelioState(ctx.shipHelioR, ctx.shipHelioV);

    tickMcc();
    tickBplane();
    tickApproach();

    // Keep MOI ignition ET live while Armed so MCCs don't desync the burn point.
    if (m_moiIgnET > m_currentET)
        m_moiBurnCtrl.updateIgnitionET(m_moiIgnET, m_eventQueue);

    // SOI transition announcements.
    {
        const bool nowInSoi = inArrivalSoi();
        if (nowInSoi && !m_wasInArrivalSoi) {
            obc::speak("Entered " + m_arrBodyName + " sphere of influence.");
            m_soiEntryPending = true;
        }
        m_wasInArrivalSoi = nowInSoi;
    }

    m_burnCtrl.tick(m_currentET, m_shipR, m_shipV);
    m_moiBurnCtrl.tick(m_currentET, m_shipArrR, m_shipArrV);

    // Re-schedule MCC event if the solution moved by more than 60 s.
    if (m_eventQueue && m_detail.valid) {
        const double mccET = m_currentET;  // MCC is "now" — tickMcc solved from current pos
        // Use arrival ET as a proxy: MCC event = current time (burn is ~now on coasting page).
        // More precisely: the MCC burn should happen as soon as possible; schedule it
        // only once when detail is first valid and re-schedule if time shifts >60 s.
        (void)mccET; // MCC scheduling handled via onRight — see below
    }
}
