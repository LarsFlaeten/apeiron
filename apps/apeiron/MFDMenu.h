#pragma once

#include "MFD.h"

#include <vector>

// ---------------------------------------------------------------------------
// MFDMenu — MFDApp that shows a selectable grid of registered MFD apps.
//
// Register apps with addApp() before the main loop.
// MFDPanel calls consumeSelection() after each frame to check if the user
// clicked a tile; if so, it switches the panel's active app and closes the menu.
// ---------------------------------------------------------------------------
class MFDMenu : public MFDApp {
public:
    const char* name() const override { return "MENU"; }

    // Register an app tile.  label is the short name shown in the tile (e.g. "ORB").
    void addApp(MFDApp* app, const char* label);

    // Returns the app the user clicked this frame (and clears the selection).
    // Returns nullptr if nothing was clicked.
    MFDApp* consumeSelection();

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

private:
    struct Entry { MFDApp* app; const char* label; };
    std::vector<Entry> m_entries;
    MFDApp*            m_selected = nullptr;
};
