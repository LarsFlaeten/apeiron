#pragma once

#include "MFD.h"

#include "apeiron/spacecraft/Porkchop.h"
#include "apeiron/spacecraft/Lambert.h"

#include <astro/Time.h>
#include <astro/State.h>

#include <glm/glm.hpp>
#include <string>

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

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    const char* rightLabel(int slot) const override;
    void        onLeft    (int slot) override;
    void        onRight   (int slot) override;

private:
    void renderPorkchop(ImDrawList* dl, ImVec2 origin, ImVec2 size);
    void renderDetail  (ImDrawList* dl, ImVec2 origin, ImVec2 size);

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
    int  m_page   = 0;   // 0 = porkchop, 1 = detail

    // ---- Cached detail for selected cell ----
    struct SelDetail {
        bool       valid    = false;
        double     depET    = 0.0;
        double     arrET    = 0.0;
        double     tofSec   = 0.0;
        float      dv1      = 0.0f;  // km/s departure ΔV
        float      dv2      = 0.0f;  // km/s arrival  ΔV
        float      c3       = 0.0f;  // km²/s²  departure C3 = |v∞|²
        glm::dvec3 depPos;            // Earth heliocentric position (km)
        glm::dvec3 arrPos;            // Mars  heliocentric position (km)
        glm::dvec3 vDepBody;          // Earth heliocentric velocity (km/s)
        glm::dvec3 vArrBody;          // Mars  heliocentric velocity (km/s)
        glm::dvec3 vDep;              // Lambert departure velocity (km/s)
        glm::dvec3 vArr;              // Lambert arrival  velocity (km/s)
    } m_detail;

    static constexpr double kDay = 86400.0;
};
