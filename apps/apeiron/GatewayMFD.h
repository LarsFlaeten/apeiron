#pragma once

#include "MFD.h"
#include "MFDContext.h"
#include "BurnController.h"

#include "apeiron/spacecraft/Porkchop.h"
#include "apeiron/spacecraft/Lambert.h"
#include "apeiron/spacecraft/VehicleConfig.h"

#include <astro/Time.h>
#include <astro/State.h>

#include <glm/glm.hpp>
#include <string>
#include <limits>

// ---------------------------------------------------------------------------
// Snapshot for quicksave / quickload — mirrors CislunarPlanSnapshot.
// ---------------------------------------------------------------------------
struct GatewayPlanSnapshot {
    bool                       valid   = false;
    spacecraft::PorkchopParams params;   // includes departureR/V override
    int                        selDep  = -1;
    int                        selTof  = -1;
    int                        parkIdx = 1;
};

// ---------------------------------------------------------------------------
// GatewayMFD — Earth → Gateway (NRHO) transfer planning.
//
// Structurally mirrors CislunarMFD but targets Gateway specifically
// using vehicleStateAtEt() for arrival instead of the SPICE Moon body.
// dv2 on the porkchop is the direct NRHO rendezvous ΔV, not a v∞.
//
// Page 0 — Window   porkchop (departure × TOF, 90 day × 3-8 day)
// Page 1 — Plan     transfer summary (dates, ΔV, NRHO INS preview)
// Page 2 — Burn     TLI burn execution (identical to CislunarMFD)
// Page 3 — Coast    transit progress + TCM
// Page 4 — NRHO     Gateway approach display
//
// Page 0 buttons:
//   Left  0: DEP<   shift departure window back 15 days
//   Left  1: DEP>   shift departure window fwd  15 days
//   Left  2: TOF<   decrease min TOF by 6 h
//   Left  3: TOF>   increase max TOF by 6 h
//   Left  4: PC     toggle plane-change ΔV contour overlay
//   Left  5: RST    reset window to now+90d, default TOF, recompute
//   Right 0: COMP / INFO
//   Right 1: WIN<   Right 2: WIN>   Right 3: RNG<   Right 4: RNG>
//
// Page 1 buttons:
//   Left  3: ALT    cycle parking orbit altitude
//   Left  4: BACK
//   Right 4: BURN
//
// Page 2 buttons:
//   Left  2: ARM / DSARM
//   Left  3: ALT
//   Left  4: BACK
//   Right 4: CST
//
// Page 3 buttons:
//   Left  4: BACK
//   Right 4: NRHO
//
// Page 4 buttons:
//   Left  4: BACK
// ---------------------------------------------------------------------------
class GatewayMFD : public MFDApp {
public:
    const char* name() const override { return "NRHO"; }

    // Supply Gateway's VehicleConfig once at startup.
    void setTargetConfig(const spacecraft::VehicleConfig* vc) { m_gatewayConfig = vc; }

    GatewayPlanSnapshot getPlan() const;
    void                restorePlan(const GatewayPlanSnapshot& snap);

    void update(const MFDContext& ctx);

    bool requestMainEngine() const { return m_burnCtrl.requestMainEngine(); }

    glm::dvec3 getTcmDv()          const { return m_tcmValid ? m_tcmDv : glm::dvec3(0.0); }
    glm::dvec3 getActiveBurnDir()  const { return m_burnCtrl.getActiveBurnDirection(); }

    bool consumeSoiEntry() {
        const bool v = m_soiEntryPending;
        m_soiEntryPending = false;
        return v;
    }

    double maxSimSpeed() const { return m_burnCtrl.maxSimSpeed(); }

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    const char* rightLabel(int slot) const override;
    void        onLeft    (int slot) override;
    void        onRight   (int slot) override;

private:
    void renderWindow(ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderPlan  (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderBurn  (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderCoast (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderNRHO  (ImDrawList* dl, ImVec2 origin, ImVec2 size);

    void   resolveSelected();
    void   compute();
    void   computePlaneGrid();
    double computeTliIgnitionET() const;

    static ImU32 dvColor(float t);

    // ---- Gateway VehicleConfig (set once by main.cpp at startup) ----
    const spacecraft::VehicleConfig* m_gatewayConfig = nullptr;

    // ---- Porkchop state ----
    spacecraft::PorkchopParams m_params;
    spacecraft::PorkchopData   m_data;
    bool                       m_hasData   = false;
    bool                       m_computing = false;
    std::string                m_error;

    // ---- Plane-change ΔV grid ----
    std::vector<float> m_planeGrid;
    float              m_planeMin = 1e9f;
    float              m_planeMax = 0.0f;
    bool               m_showPC  = false;

    // ---- Selection ----
    int m_selDep = -1;
    int m_selTof = -1;
    int m_page   = 0;

    // ---- Cached detail for selected cell ----
    struct SelDetail {
        bool       valid    = false;
        double     depET    = 0.0;
        double     arrET    = 0.0;
        double     tofSec   = 0.0;
        float      dv1      = 0.0f;   // TLI ΔV (km/s)
        float      dv2      = 0.0f;   // NRHO insertion ΔV (km/s)
        float      c3       = 0.0f;
        glm::dvec3 depPos;
        glm::dvec3 arrPos;            // Gateway ECI position at arrival
        glm::dvec3 vDep;
        glm::dvec3 vArr;
        glm::dvec3 vDepBody;          // parking orbit velocity at burn point
        glm::dvec3 vArrBody;          // Gateway ECI velocity at arrival
    } m_detail;

    // Parking orbit altitude (km above Earth surface)
    static constexpr double kParkAlts[] = { 200.0, 400.0, 600.0, 1000.0 };
    int m_parkIdx = 1;  // default 400 km

    static constexpr double kGmEarth = 398600.4418;
    static constexpr double kGmMoon  =   4902.800118;
    static constexpr double kREarth  =   6371.0;
    static constexpr double kRMoon   =   1737.4;
    static constexpr double kDay     =  86400.0;

    // ---- Ship state ----
    glm::dvec3 m_shipR     { 0.0 };
    glm::dvec3 m_shipV     { 0.0 };
    double     m_currentET   = 0.0;
    double     m_mainThrustN = 25700.0;
    double     m_shipMass    = 26500.0;

    // ---- Gateway petaloid trace (ECI, sampled ±1 NRHO period around arrival) ----
    // Precomputed in resolveSelected(); used by renderCoast for the rosette overlay.
    std::vector<glm::dvec3> m_gwTrace;      // Gateway ECI positions
    std::vector<glm::dvec3> m_moonOrbitPts; // Moon ECI positions (same sample times)

    // ---- Coast TCM ----
    glm::dvec3 m_tcmDv    { 0.0 };
    bool       m_tcmValid = false;

    // ---- Moon SOI state ----
    glm::dvec3 m_shipMoonR { 0.0 };
    glm::dvec3 m_shipMoonV { 0.0 };
    bool       m_inMoonSoi    = false;
    bool       m_wasInMoonSoi = false;
    bool       m_soiEntryPending = false;

    // ---- Gateway approach state (valid when m_inMoonSoi && m_gatewayOk) ----
    glm::dvec3 m_gatewayMoonR { 0.0 };  // Gateway Moon-centric position
    glm::dvec3 m_gatewayMoonV { 0.0 };  // Gateway Moon-centric velocity
    bool       m_gatewayOk    = false;

    // Zoom-box for porkchop page.
    ImVec2 m_zoomStart  { 0.0f, 0.0f };
    bool   m_zoomActive = false;

    glm::dmat3 m_windowViewRot { 1.0 };
    glm::dmat3 m_planViewRot   { 1.0 };
    glm::dmat3 m_burnViewRot   { 1.0 };
    glm::dmat3 m_coastViewRot  { 1.0 };
    glm::dmat3 m_nrhoViewRot   { 1.0 };

    OBCEventQueue*         m_eventQueue = nullptr;
    spacecraft::Autopilot* m_autopilot  = nullptr;

    BurnController m_burnCtrl;  // TLI burn
};
