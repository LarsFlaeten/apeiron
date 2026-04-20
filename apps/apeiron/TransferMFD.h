#pragma once

#include "MFD.h"

#include "apeiron/spacecraft/Porkchop.h"
#include "apeiron/spacecraft/Lambert.h"

#include <astro/Time.h>
#include <astro/State.h>

#include <glm/glm.hpp>
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
// Page 0 — Porkchop plot  (departure date × TOF grid, colour = total ΔV)
// Page 1 — Leg detail     (orbit diagram + transfer parameters for selected cell)
//
// Buttons:
//   Left  0: DEP<   shift departure window back  30 days
//   Left  1: DEP>   shift departure window fwd   30 days
//   Left  2: TOF<   shift TOF range down          30 days
//   Left  3: TOF>   shift TOF range up             30 days
//   Left  4: COMP   recompute the grid  (page 0) / back to plot (page 1)
//   Right 0: WIN<   halve departure window span
//   Right 1: WIN>   double departure window span
//   Right 2: RNG<   halve TOF range span
//   Right 3: RNG>   double TOF range span
//   Right 4: ---    (reserved for future)
//
// Click a grid cell to select it; if on page 0 the right side of page 0
// shows a summary and pressing the cell again (or a dedicated button) opens
// page 1.
// ---------------------------------------------------------------------------
class TransferMFD : public MFDApp {
public:
    const char* name() const override { return "XFER"; }

    // Set the current simulation ET — used to centre the departure window.
    void setEpoch(const astro::EphemerisTime& et);

    // Trigger a (synchronous) grid recompute.
    void compute();

    // Feed current ship state (geocentric ECLIPJ2000, km / km/s) and sim time each frame.
    void updateShipState(const glm::dvec3& r, const glm::dvec3& v, double muEarth,
                         double currentET)
    {
        m_shipR     = r;
        m_shipV     = v;
        m_muEarth   = muEarth;
        m_currentET = currentET;
    }

    // Feed current propulsion parameters — used for burn timing on page 2.
    void updateBurnParams(double mainThrustN, double massKg)
    {
        m_mainThrustN = mainThrustN;
        m_shipMass    = massKg;
    }

    // Feed current heliocentric ship state (Sun-centred ECLIPJ2000, km / km/s).
    // Called from main.cpp alongside updateShipState.
    void updateHelioState(const glm::dvec3& r, const glm::dvec3& v)
    {
        m_shipHelioR = r;
        m_shipHelioV = v;
    }

    // Save / restore the plan state (used by SimSave and explicit plan save/load).
    // restorePlan re-resolves the selected cell so the detail page is ready.
    TransferPlanSnapshot getPlan() const;
    void                 restorePlan(const TransferPlanSnapshot& snap);

    // Recomputes the MCC ΔV vector from current ship state and plan.
    // Call every frame from main.cpp regardless of which view is active.
    void tickMcc();

    // Recomputes the B-plane targeting correction whenever the ship is on a
    // hyperbolic approach to Mars.  Call every frame alongside tickMcc().
    void tickBplane();

    // Returns the B-plane ΔV vector (ECLIPJ2000 inertial, km/s).
    // Zero when not on a hyperbolic Mars approach or too far from Mars.
    glm::dvec3 getBplaneDv() const { return m_bplaneDv; }
    double     getBplanePeCurrentKm() const { return m_bplanePeCurrentKm; }

    // Returns the current MCC ΔV vector (heliocentric ECLIPJ2000, km/s).
    // Zero when no plan is set or no valid Lambert solution exists.
    glm::dvec3 getMccDv() const { return m_mccDv; }

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    const char* rightLabel(int slot) const override;
    void        onLeft    (int slot) override;
    void        onRight   (int slot) override;

private:
    void renderPorkchop (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderDetail   (ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderDeparture(ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderCoasting (ImDrawList* dl, ImVec2 origin, ImVec2 size);

    // Re-solve Lambert for the selected cell and cache the result.
    void resolveSelected();

    // Color-map: 0=green(low ΔV) → 1=red(high ΔV).
    static ImU32 dvColor(float t);

    // ---- Porkchop state ----
    spacecraft::PorkchopParams m_params;
    spacecraft::PorkchopData   m_data;
    bool                       m_hasData   = false;
    bool                       m_computing = false;
    std::string                m_error;

    // ---- Selection ----
    int  m_selDep = -1;
    int  m_selTof = -1;
    int  m_page   = 0;   // 0 = porkchop, 1 = detail, 2 = departure, 3 = coasting

    // ---- Cached detail for selected cell ----
    struct SelDetail {
        bool       valid    = false;
        double     depET    = 0.0;
        double     arrET    = 0.0;
        double     tofSec   = 0.0;
        float      dv1      = 0.0f;  // km/s departure ΔV (heliocentric)
        float      dv2      = 0.0f;  // km/s arrival  ΔV (heliocentric)
        float      c3       = 0.0f;  // km²/s²  departure C3 = |v∞|²
        glm::dvec3 depPos;            // Earth heliocentric position (km)
        glm::dvec3 arrPos;            // Mars  heliocentric position (km)
        glm::dvec3 vDepBody;          // Earth heliocentric velocity (km/s)
        glm::dvec3 vArrBody;          // Mars  heliocentric velocity (km/s)
        glm::dvec3 vDep;              // Lambert departure velocity (km/s)
        glm::dvec3 vArr;              // Lambert arrival  velocity (km/s)
        // Orbital state of departure body (for ellipse drawing).
        glm::dvec3 depBodyVel;        // same as vDepBody — kept for clarity
        glm::dvec3 arrBodyVel;        // same as vArrBody
    } m_detail;

    // Parking orbit radius for TMI burn computation (km from Earth centre).
    // Cycled by left button 5 (ALT).
    static constexpr double kParkAlts[] = { 6778.0, 6978.0, 7378.0, 8378.0 };
    // 400 km, 600 km, 1000 km, 2000 km altitude (Earth R=6378 km)
    int m_parkIdx = 0;   // index into kParkAlts

    static constexpr double kDay = 86400.0;

    // Persistent view rotations (right-drag to tumble, double-right-click resets).
    glm::dmat3 m_detailViewRot  { 1.0 };   // page 1: heliocentric
    glm::dmat3 m_depViewRot     { 1.0 };   // page 2: geocentric departure
    glm::dmat3 m_coastViewRot   { 1.0 };   // page 3: heliocentric coasting

    // Current ship geocentric state and sim time (fed by main.cpp each frame).
    glm::dvec3 m_shipR     { 0.0 };
    glm::dvec3 m_shipV     { 0.0 };
    double     m_muEarth   = 398600.4418;
    double     m_currentET = 0.0;

    // Propulsion parameters for burn timing (fed by main.cpp each frame).
    double     m_mainThrustN = 25700.0;   // N
    double     m_shipMass    = 26500.0;   // kg

    // Heliocentric ship state (Sun-centred ECLIPJ2000, km / km/s).
    // Computed in main.cpp from Earth heliocentric + geocentric ship state.
    glm::dvec3 m_shipHelioR { 0.0 };
    glm::dvec3 m_shipHelioV { 0.0 };

    // Most recent MCC ΔV vector (heliocentric ECLIPJ2000, km/s).
    // Written by renderCoasting() each frame; zero when no valid solution.
    glm::dvec3 m_mccDv { 0.0 };

    // B-plane targeting (periapsis altitude at Mars).
    // kPeTargetAlts: selectable target periapsis altitudes (km above Mars surface).
    static constexpr double kPeTargetAlts[] = { 200.0, 500.0, 1000.0, 2000.0, 5000.0 };
    int        m_peAltIdx         = 0;       // index into kPeTargetAlts
    glm::dvec3 m_bplaneDv          { 0.0 };   // ΔV vector, ECLIPJ2000 km/s
    double     m_bplanePeCurrentKm = 0.0;    // projected periapsis altitude (km, may be <0 = impact)
    bool       m_bplaneValid       = false;
    bool       m_capturedAtArrival = false;   // true when in elliptic orbit around arrival body


};
