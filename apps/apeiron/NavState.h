#pragma once
#include "NavMode.h"

// Mutable navigation/docking state shared between main loop and UI renderers.
struct NavState {
    NavMode   navMode    = NavMode::Orbit;
    TransMode transMode  = TransMode::Fine;
    int       dockTgtIdx = -1;  // -1 = none; ≥1 = index into spacecraft[]
};
