#pragma once

#include "MFD.h"
#include "MFDContext.h"
#include "BurnController.h"

#include "apeiron/spacecraft/Porkchop.h"
#include "apeiron/spacecraft/Lambert.h"

#include <astro/Time.h>
#include <astro/State.h>

#include <glm/glm.hpp>
#include <string>
#include <limits>

// ---------------------------------------------------------------------------
// Serialisable snapshot — saved/restored with the scenario.
// The porkchop grid is recomputed on restore; only the parameters and
// selected-cell indices need to be persisted.
// ---------------------------------------------------------------------------
struct CislunarPlanSnapshot {
    bool                       valid     = false;
    spacecraft::PorkchopParams params;    // includes departureR/V override
    int                        selDep    = -1;
    int                        selTof    = -1;
    int                        parkIdx   = 1;
    int                        loiPeIdx  = 0;
};

// ---------------------------------------------------------------------------
// CislunarMFD — Earth-Moon transfer planning display.
//
// Page 0 — Window   porkchop plot (departure × TOF, 90 day × 3–8 day)
// Page 1 — Plan     transfer summary (dates, ΔV, LOI preview)
// Page 2 — Burn     TLI burn execution (mirrors TransferMFD departure page)
// Page 3 — Coast    transit progress bar + distance to Moon
// Page 4 — LOI      lunar insertion burn planning + ARM
//
// Page 0 buttons:
//   Left  0: DEP<   shift departure window back 15 days
//   Left  1: DEP>   shift departure window fwd  15 days
//   Left  2: TOF<   decrease min TOF by 6 h
//   Left  3: TOF>   increase max TOF by 6 h
//   Left  4: PC     toggle plane-change ΔV contour overlay
//   Left  5: RST    reset window to now+90d, default TOF, recompute
//   Right 0: COMP   recompute (or INFO when a cell is selected)
//   Right 1: WIN<   halve departure window span
//   Right 2: WIN>   double departure window span
//   Right 3: RNG<   halve TOF span
//   Right 4: RNG>   double TOF span
//
// Page 1 buttons:
//   Left  3: ALT    cycle parking orbit altitude
//   Left  4: BACK   back to porkchop
//   Right 4: BURN   go to burn page
//
// Page 2 buttons:
//   Left  2: ARM / DSARM
//   Left  3: ALT    cycle parking orbit altitude
//   Left  4: BACK   back to plan
//   Right 4: CST    go to coast page
//
// Page 3 buttons:
//   Left  4: BACK   back to burn
//   Right 4: LOI    go to LOI page
//
// Page 4 buttons:
//   Left  3: PE     cycle lunar Pe target altitude
//   Left  4: BACK   back to coast
//   Right 4: ARM / DSARM
// ---------------------------------------------------------------------------
class CislunarMFD : public MFDApp {
public:
    const char* name() const override { return "LUNAR"; }

    void update(const MFDContext& ctx);

    CislunarPlanSnapshot getPlan() const;
    void                 restorePlan(const CislunarPlanSnapshot& snap);

    bool requestMainEngine() const { return m_burnCtrl.requestMainEngine() || m_loiBurnCtrl.requestMainEngine(); }

    // TCM course-correction vector (geocentric, km/s). Zero when not coasting
    // or when the correction is negligible. Fed to the nav-console TCM autopilot.
    glm::dvec3 getTcmDv() const { return m_tcmValid ? m_tcmDv : glm::dvec3(0.0); }

    // Returns true once on the rising edge of Moon SOI entry, then resets.
    // main.cpp uses this to drop time-acceleration to 1× so the pilot can
    // arm the LOI burn.
    bool consumeSoiEntry() {
        const bool v = m_soiEntryPending;
        m_soiEntryPending = false;
        return v;
    }

    double maxSimSpeed() const {
        const double a = m_burnCtrl.maxSimSpeed();
        const double b = m_loiBurnCtrl.maxSimSpeed();
        return (a > 0.0) ? a : b;
    }

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    const char* rightLabel(int slot) const override;
    void        onLeft    (int slot) override;
    void        onRight   (int slot) override;

private:
    void renderWindow (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderPlan   (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderBurn   (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderCoast  (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderLOI    (ImDrawList* dl, ImVec2 origin, ImVec2 size);

    void resolveSelected();
    void compute();
    void computePlaneGrid();

    double computeTliIgnitionET() const;
    double computeLoiIgnitionET() const; // live Pe time from Moon-centric hyperbolic anomaly

    static ImU32 dvColor(float t);

    // ---- Porkchop state ----
    spacecraft::PorkchopParams m_params;
    spacecraft::PorkchopData   m_data;
    bool                       m_hasData   = false;
    bool                       m_computing = false;
    std::string                m_error;

    // ---- Plane-change ΔV grid (same cell indexing as m_data) ----
    // Computed after each porkchop from the ship's current parking orbit normal.
    // Each cell: approximate plane-change ΔV (km/s) for that departure/TOF combo.
    std::vector<float> m_planeGrid;
    float              m_planeMin  = 1e9f;
    float              m_planeMax  = 0.0f;
    bool               m_showPC    = false;   // overlay plane-change contours on porkchop

    // ---- Selection ----
    int m_selDep = -1;
    int m_selTof = -1;
    int m_page   = 0;   // 0=window, 1=plan, 2=burn, 3=coast, 4=LOI

    // ---- Cached detail for selected cell ----
    struct SelDetail {
        bool       valid    = false;
        double     depET    = 0.0;   // Lambert departure epoch (porkchop grid time)
        double     arrET    = 0.0;
        double     tofSec   = 0.0;
        float      dv1      = 0.0f;  // TLI ΔV (km/s)
        float      dv2      = 0.0f;  // Moon-relative arrival v∞ (km/s)
        float      c3       = 0.0f;  // geocentric C3 (km²/s²)
        glm::dvec3 depPos;
        glm::dvec3 arrPos;
        glm::dvec3 vDep;             // geocentric Lambert departure velocity
        glm::dvec3 vArr;             // geocentric Lambert arrival velocity
        glm::dvec3 vDepBody;         // geocentric ship velocity at departure (override)
        glm::dvec3 vArrBody;         // Moon velocity at arrival
    } m_detail;

    // Parking orbit altitude (km above Earth surface)
    static constexpr double kParkAlts[]  = { 200.0, 400.0, 600.0, 1000.0 };
    int m_parkIdx = 1;   // default 400 km

    // LOI Pe target altitude (km above Moon surface)
    static constexpr double kLoiPeAlts[] = { 100.0, 200.0, 500.0, 1000.0 };
    int m_loiPeIdx = 0;  // default 100 km

    // LOI insertion inclination presets (ecliptic, deg). -1 = FREE (Pe only).
    static constexpr double kLoiIncDeg[] =
        { -1.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0 };
    static constexpr const char* kLoiIncName[] =
        { "FREE", "PRO 0", "30", "60", "POL 90", "120", "150", "RET 180" };
    int    m_loiIncIdx          = 0;   // 0 = FREE
    double m_insertionIncDeg    = 0.0; // current predicted insertion inclination (deg, ecliptic)
    double m_minAchievableIncDeg = 0.0;

    static constexpr double kGmEarth = 398600.4418; // km³/s²
    static constexpr double kGmMoon  = 4902.800118;  // km³/s²
    static constexpr double kREarth  = 6371.0;       // km
    static constexpr double kRMoon   = 1737.4;       // km
    static constexpr double kDay     = 86400.0;

    // ---- Ship state (geocentric ECI, km / km/s) ----
    glm::dvec3 m_shipR     { 0.0 };
    glm::dvec3 m_shipV     { 0.0 };
    double     m_currentET = 0.0;

    double m_mainThrustN = 25700.0;
    double m_shipMass    = 26500.0;

    // ---- Coast TCM state ----
    // Re-solved every frame from current geocentric state to planned Moon arrival.
    glm::dvec3 m_tcmDv    { 0.0 };   // km/s correction vector (geocentric)
    bool       m_tcmValid = false;

    // ---- LOI approach state (Moon-centric) ----
    glm::dvec3 m_shipMoonR    { 0.0 };
    glm::dvec3 m_shipMoonV    { 0.0 };
    bool       m_inMoonSoi    = false;
    double     m_moonPeCurrent = std::numeric_limits<double>::quiet_NaN(); // km alt, from B-plane
    bool       m_wasInMoonSoi = false;  // previous-frame SOI state for edge detection
    bool       m_soiEntryPending = false; // one-shot: consumed by main to drop to 1×

    // Zoom-box state for porkchop page (right-drag to draw).
    ImVec2 m_zoomStart  { 0.0f, 0.0f };
    bool   m_zoomActive = false;

    glm::dmat3 m_windowViewRot { 1.0 };
    glm::dmat3 m_planViewRot   { 1.0 };
    glm::dmat3 m_burnViewRot   { 1.0 };
    glm::dmat3 m_coastViewRot  { 1.0 };
    glm::dmat3 m_loiViewRot    { 1.0 };

    OBCEventQueue*         m_eventQueue = nullptr;
    spacecraft::Autopilot* m_autopilot  = nullptr;

    BurnController m_burnCtrl;     // TLI burn
    BurnController m_loiBurnCtrl;  // LOI burn
};
