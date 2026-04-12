#pragma once

#include "MFD.h"

#include "apeiron/spacecraft/Porkchop.h"

#include <astro/Time.h>

// ---------------------------------------------------------------------------
// TransferMFD — interplanetary transfer planning display.
//
// Shows a porkchop plot (departure date × TOF grid) for a single-leg
// Lambert transfer.  Default target: Earth → Mars.
//
// Buttons:
//   Left  0: DEP<   shift departure window back  30 days
//   Left  1: DEP>   shift departure window fwd   30 days
//   Left  2: TOF<   shift TOF range down          30 days
//   Left  3: TOF>   shift TOF range up             30 days
//   Left  4: COMP   recompute the grid
//   Right 0: WIN<   halve departure window span
//   Right 1: WIN>   double departure window span
//   Right 2: RNG<   halve TOF range span
//   Right 3: RNG>   double TOF range span
//
// Call setEpoch() once from main.cpp to initialise the time window around
// the current simulation time.
// ---------------------------------------------------------------------------
class TransferMFD : public MFDApp {
public:
    const char* name() const override { return "XFER"; }

    // Set the current simulation ET — used to centre the departure window.
    void setEpoch(const astro::EphemerisTime& et);

    // Trigger a (synchronous) grid recompute.  Safe to call from the main thread.
    void compute();

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    const char* rightLabel(int slot) const override;
    void        onLeft    (int slot) override;
    void        onRight   (int slot) override;

private:
    // Color-map: 0=green(low ΔV) → 1=red(high ΔV).
    static ImU32 dvColor(float t);

    spacecraft::PorkchopParams m_params;
    spacecraft::PorkchopData   m_data;
    bool                       m_hasData = false;
    bool                       m_computing = false;

    // Hover / selection state.
    int   m_selDep = -1;
    int   m_selTof = -1;

    static constexpr double kDay = 86400.0;  // seconds
};
