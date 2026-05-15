#pragma once

#include "MFD.h"
#include "MFDContext.h"
#include "BurnController.h"
#include "MccBurnController.h"

#include "apeiron/spacecraft/Porkchop.h"
#include "apeiron/spacecraft/Lambert.h"

#include <astro/Time.h>
#include <astro/State.h>

#include <glm/glm.hpp>
#include <limits>
#include <string>

// ---------------------------------------------------------------------------
// Serialisable snapshot of the TransferMFD plan — used by SimSave.
// Contains just the parameters needed to reconstruct the full display;
// the porkchop grid and orbit diagram are re-derived on restore.
// ---------------------------------------------------------------------------
struct TransferPlanSnapshot {
    bool                        valid   = false;
    spacecraft::PorkchopParams  params;       // window / TOF / body config
    int                         selDep  = -1; // selected column in grid
    int                         selTof  = -1; // selected row in grid
    int                         parkIdx = 0;  // parking altitude index
};

// ---------------------------------------------------------------------------
// TransferMFD — interplanetary transfer planning display.
//
// Page 4 — Body select  (heliocentric diagram, cycle DEP/ARR through planets)
// Page 0 — Porkchop plot  (departure date × TOF grid, colour = total ΔV)
// Page 1 — Leg detail     (orbit diagram + transfer parameters for selected cell)
// Page 2 — Departure      (geocentric burn planning)
// Page 3 — Coasting       (heliocentric progress, MCC, B-plane summary)
// Page 5 — Arrival        (B-plane full targeting: Pe + inclination, MOI ARM)
//
// Page 0 buttons:
//   Left  0: DEP<   shift departure window back  30 days
//   Left  1: DEP>   shift departure window fwd   30 days
//   Left  2: TOF<   shift TOF range down          30 days
//   Left  3: TOF>   shift TOF range up             30 days
//   Left  4: BDY    open body-select page
//   Left  5: RST    reset window to now + 2yr, default TOF, recompute
//   Right 0: COMP   recompute (or INFO when a cell is selected)
//   Right 1: WIN<   halve departure window span
//   Right 2: WIN>   double departure window span
//   Right 3: RNG<   halve TOF range span
//   Right 4: RNG>   double TOF range span
//
// Page 3 buttons:
//   Left  3: PE     cycle Pe altitude target
//   Left  4: BACK
//   Right 4: ARR    open arrival page
//
// Page 4 buttons:
//   Left  0: DEP<   cycle departure body backward
//   Left  1: DEP>   cycle departure body forward
//   Left  2: ARR<   cycle arrival body backward
//   Left  3: ARR>   cycle arrival body forward
//   Left  4: OK     return to porkchop
//
// Page 5 buttons:
//   Left  0: INC<   cycle target inclination preset backward
//   Left  1: INC>   cycle target inclination preset forward
//   Left  4: BACK
//   Right 3: PE     cycle Pe altitude target
//   Right 4: ARM / DSARM
// ---------------------------------------------------------------------------
class TransferMFD : public MFDApp {
public:
    const char* name() const override { return "XFER"; }

    // Set the current simulation ET — used to centre the departure window.
    void setEpoch(const astro::EphemerisTime& et);

    // Trigger a (synchronous) grid recompute.
    void compute();

    // Feed current ship state (departure-body-centric ECLIPJ2000, km / km/s) each frame.
    void updateShipState(const glm::dvec3& r, const glm::dvec3& v, double currentET)
    {
        m_shipR     = r;
        m_shipV     = v;
        m_currentET = currentET;
    }

    // Feed current propulsion parameters — used for burn timing on page 2.
    void updateBurnParams(double mainThrustN, double massKg)
    {
        m_mainThrustN = mainThrustN;
        m_shipMass    = massKg;
    }

    // Feed current heliocentric ship state (Sun-centred ECLIPJ2000, km / km/s).
    void updateHelioState(const glm::dvec3& r, const glm::dvec3& v)
    {
        m_shipHelioR = r;
        m_shipHelioV = v;
    }

    // Save / restore the plan state (used by SimSave and explicit plan save/load).
    TransferPlanSnapshot getPlan() const;
    void                 restorePlan(const TransferPlanSnapshot& snap);

    // Unified per-frame update from MFDContext — preferred entry point.
    void update(const MFDContext& ctx);

    // Recomputes the MCC ΔV vector from current ship state and plan.
    void tickMcc();

    // Recomputes B-plane targeting correction on hyperbolic approach to arrival body.
    void tickBplane();

    // Computes time-to-Pe, MOI burn duration/ignition ET from cached B-plane state.
    void tickApproach();

    // Returns true once per SOI entry event, then resets.  main.cpp uses this to drop to 1×.
    bool consumeSoiEntry() { const bool v = m_soiEntryPending; m_soiEntryPending = false; return v; }

    glm::dvec3 getBplaneDv() const { return m_bplaneDv; }
    double     getBplanePeCurrentKm() const { return m_bplanePeCurrentKm; }
    glm::dvec3 getMccDv() const { return m_mccDv; }

    // Transfer progress [0, 1]; returns -1 if no plan is loaded.
    double getTransferProgress() const {
        if (!m_detail.valid || m_detail.tofSec <= 0.0) return -1.0;
        return (m_currentET - m_detail.depET) / m_detail.tofSec;
    }

    // Returns true when either burn autopilot wants the main engine firing.
    // OR this with the SPACEBAR key in main.cpp.
    bool   requestMainEngine() const {
        return m_burnCtrl.requestMainEngine() || m_moiBurnCtrl.requestMainEngine();
    }

    // MCC burn AP — arm, tick, and query.
    void         armMccBurn(bool coarse) { m_mccBurnCtrl.arm(coarse); }
    void         disarmMccBurn()         { m_mccBurnCtrl.disarm(); }
    // Per-frame tick: returns true when burn completes (Done state).
    // Caller must engage Killrot and call consumeMccDone().
    bool         tickMccBurn(double mccDvMs, double attErrorRad)
                     { return m_mccBurnCtrl.tick(mccDvMs, attErrorRad); }
    bool         requestMccMainEngine() const { return m_mccBurnCtrl.requestMainEngine(); }
    bool         requestMccAuxEngine()  const { return m_mccBurnCtrl.requestAuxEngine(); }
    void         consumeMccDone()             { m_mccBurnCtrl.consumeDone(); }
    MccBurnPhase mccBurnPhase()  const { return m_mccBurnCtrl.phase; }
    bool         mccBurnCoarse() const { return m_mccBurnCtrl.coarse; }
    bool         mccBurnActive() const { return m_mccBurnCtrl.active(); }

    // Returns a positive time-accel cap during execution or MCC warning, or 0.
    // 1× while an MCC deviation warning is active; 10× while a burn is executing.
    double maxSimSpeed() const {
        if (m_mccWarnActive) return 1.0;
        const double a = m_burnCtrl.maxSimSpeed();
        const double b = m_moiBurnCtrl.maxSimSpeed();
        return (a > 0.0) ? a : b;
    }

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    const char* rightLabel(int slot) const override;
    void        onLeft    (int slot) override;
    void        onRight   (int slot) override;

private:
    void renderBodySelect (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderPorkchop   (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderDetail     (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderDeparture  (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderCoasting   (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderArrival    (ImDrawList* dl, ImVec2 origin, ImVec2 size);

    void resolveSelected();
    static ImU32 dvColor(float t);

    // Computes the ignition ET for the next burn-TA passage given the current
    // ship state and selected transfer plan.  Returns 0.0 on failure (degenerate
    // orbit, no plan, etc.).
    double computeIgnitionET() const;

    // Queries SPICE for departure body radius, GM, and SOI radius.
    // Uses m_params.departureBody / centralBody and m_currentET.
    void queryDepBodyConstants();

    // Resets TOF range to a sensible default based on the current dep/arr bodies.
    void updateDefaultTof();

    // Select a planet by index into kPlanets[] for departure or arrival.
    void selectDeparture(int idx);
    void selectArrival  (int idx);

    // Returns true when the ship is inside the arrival body's sphere of influence.
    bool inArrivalSoi() const;

    // Find the index of a NAIF ID in m_bodies (-1 if not found).
    int findPlanetIdx(int naifId) const;

    // ---- Body selection ----
    // Populated from MFDContext::solarSystemBodies on first update().
    std::vector<BodyEntry> m_bodies;
    int m_depPlanetIdx = -1;   // index into m_bodies (-1 until populated)
    int m_arrPlanetIdx = -1;   // index into m_bodies (-1 until populated)

    // Display names for departure and arrival bodies.
    std::string m_depBodyName = "EARTH";
    std::string m_arrBodyName = "MARS";

    // Departure body physical constants (queried from SPICE when body changes).
    double m_depBodyRadius = 6378.0;      // km, equatorial radius
    double m_muDep         = 398600.4418; // km³/s²
    double m_depSOI        = 929000.0;    // km, sphere-of-influence radius

    // ---- Porkchop state ----
    spacecraft::PorkchopParams m_params;
    spacecraft::PorkchopData   m_data;
    bool                       m_hasData   = false;
    bool                       m_computing = false;
    std::string                m_error;

    // ---- Selection ----
    int  m_selDep = -1;
    int  m_selTof = -1;
    int  m_page   = 0;   // 0=porkchop, 1=detail, 2=departure, 3=coasting, 4=bodyselect, 5=arrival

    // ---- Cached detail for selected cell ----
    struct SelDetail {
        bool       valid    = false;
        double     depET    = 0.0;
        double     arrET    = 0.0;
        double     tofSec   = 0.0;
        float      dv1      = 0.0f;
        float      dv2      = 0.0f;
        float      c3       = 0.0f;
        glm::dvec3 depPos;
        glm::dvec3 arrPos;
        glm::dvec3 vDepBody;
        glm::dvec3 vArrBody;
        glm::dvec3 vDep;
        glm::dvec3 vArr;
        glm::dvec3 depBodyVel;
        glm::dvec3 arrBodyVel;
    } m_detail;

    // Parking orbit altitude above departure body surface (km).
    // Generic: works for any departure body.
    static constexpr double kParkAlts[] = { 400.0, 600.0, 1000.0, 2000.0 };
    int m_parkIdx = 0;

    static constexpr double kDay = 86400.0;

    glm::dmat3 m_detailViewRot  { 1.0 };
    glm::dmat3 m_depViewRot     { 1.0 };
    glm::dmat3 m_coastViewRot   { 1.0 };
    glm::dmat3 m_bodySelViewRot { 1.0 };
    glm::dmat3 m_arrViewRot     { 1.0 };

    // Current ship departure-body-centric state and sim time.
    glm::dvec3 m_shipR     { 0.0 };
    glm::dvec3 m_shipV     { 0.0 };
    double     m_currentET = 0.0;

    double     m_mainThrustN = 25700.0;
    double     m_shipMass    = 26500.0;

    glm::dvec3 m_shipHelioR { 0.0 };
    glm::dvec3 m_shipHelioV { 0.0 };

    glm::dvec3 m_mccDv { 0.0 };

    OBCEventQueue*         m_eventQueue = nullptr;
    spacecraft::Autopilot* m_autopilot  = nullptr;
    double                 m_lastMccET  = 0.0;

    BurnController    m_burnCtrl;     // departure (TMI) burn
    BurnController    m_moiBurnCtrl;  // arrival (MOI) burn
    MccBurnController m_mccBurnCtrl;  // MCC coarse/fine burn AP

    // Arrival-body-centric ship state — cached in tickBplane(), used by tickApproach().
    glm::dvec3 m_shipArrR { 0.0 };
    glm::dvec3 m_shipArrV { 0.0 };
    double     m_muArr         = 0.0;   // arrival body μ, km³/s²
    double     m_arrBodyRadius = 0.0;   // arrival body equatorial radius, km

    // Approach / MOI planning — cached in tickApproach(), used by renderCoasting() and ARM.
    double m_tToPeHyp      = std::numeric_limits<double>::quiet_NaN();
    double m_moiIgnET      = 0.0;   // absolute ET for MOI ignition (0 = not computable)
    double m_moiDvCirc     = 0.0;   // circularisation ΔV at Pe, km/s
    double m_moiBurnDur    = 0.0;   // estimated burn duration, seconds
    double m_predictedPeKm = std::numeric_limits<double>::quiet_NaN();   // finite-burn Pe prediction (0 = not computed)

    // B-plane targeting.
    static constexpr double kPeTargetAlts[] = { 200.0, 500.0, 1000.0, 2000.0, 5000.0 };
    int        m_peAltIdx         = 0;
    glm::dvec3 m_bplaneDv          { 0.0 };
    double     m_bplanePeCurrentKm = 0.0;
    bool       m_bplaneValid       = false;
    bool       m_capturedAtArrival = false;
    bool       m_wasInArrivalSoi    = false;  // previous-frame SOI state for transition detection
    bool       m_soiEntryPending    = false;  // set on rising edge; consumed by main to drop to 1×

    // Arrival inclination targeting (page 5).
    // Preset -1.0 = FREE (only Pe targeting, no inclination change).
    // Other values are target ecliptic inclination in degrees.
    static constexpr double kIncPresetDeg[] =
        { -1.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0 };
    int    m_incPresetIdx       = 0;   // 0 = FREE
    double m_insertionIncDeg    = 0.0; // current predicted insertion orbit inclination (deg)
    double m_minAchievableIncDeg = 0.0; // minimum ecliptic inc achievable via B-plane (deg)

    // MCC deviation warning (page 3 — Coasting).
    // Index 0 = OFF; otherwise the threshold in km/s at which a voice warning
    // fires and sim speed is capped to 1× until the deviation drops back below.
    static constexpr double kMccWarnKms[] = { 0.0, 0.1, 0.2, 0.5 };
    int  m_mccWarnIdx    = 0;     // 0 = OFF
    bool m_mccWarnActive = false; // true while dV exceeds threshold
};
