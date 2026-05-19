#pragma once
#include "NavMode.h"
#include "MccBurnController.h"

// Mutable navigation/docking state shared between main loop and UI renderers.
struct NavState {
    NavMode   navMode    = NavMode::Orbit;
    TransMode transMode  = TransMode::Fine;

    // Docking target — spacecraft level
    int dockTgtIdx  = -1;   // -1 = none; ≥1 = index into spacecraft[]

    // Docking port sub-selection on the target spacecraft.
    // -1 = whole spacecraft (CoM); ≥0 = index into that spacecraft's port list.
    // Only valid when dockTgtIdx >= 0.
    int dockPortIdx = -1;

    // Number of compatible ports currently available for sub-selection.
    // Updated each frame by main.cpp; used by NavConsole to show/hide the PORT button.
    int compatiblePortCount = 0;

    // MCC burn AP — one-shot toggle requests set by NavConsole/key, cleared by main.cpp.
    bool mccCoarseToggle = false;
    bool mccFineToggle   = false;
    // Current MCC burn state — written by main.cpp, read by NavConsole for display.
    MccBurnPhase mccBurnPhase  = MccBurnPhase::Idle;
    bool         mccBurnCoarse = true;
    bool         lambertFlip   = false;  // MCC suppressed: near 180° transfer angle
};
