#pragma once

#include "MFD.h"

// ---------------------------------------------------------------------------
// DockingMFD — Multifunction Display for docking approach guidance.
//
// Currently a stub; future content:
//   - lateral offset from port axis
//   - closure rate along port +X
//   - roll error
//   - port-relative attitude indicator
// ---------------------------------------------------------------------------
class DockingMFD : public MFDApp {
public:
    const char* name() const override { return "DOCK"; }

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;
};
