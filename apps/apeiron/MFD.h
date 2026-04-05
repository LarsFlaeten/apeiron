#pragma once

#include <imgui.h>

class MFDMenu;  // forward declaration — defined in MFDMenu.h

// ---------------------------------------------------------------------------
// MFDApp — abstract base for Multifunction Display applications.
//
// Subclasses implement render() to draw content via an ImDrawList, and
// leftLabel/rightLabel to label the six side buttons.  onLeft/onRight are
// called when a button is clicked.
// ---------------------------------------------------------------------------
class MFDApp {
public:
    static constexpr int kSlots = 6;

    virtual ~MFDApp() = default;

    virtual const char* name() const = 0;

    // Draw content into the rectangle [origin, origin+size).
    // Called from MFDPanel::render() with the inner content area (buttons excluded).
    virtual void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) = 0;

    virtual const char* leftLabel (int /*slot*/) const { return ""; }
    virtual const char* rightLabel(int /*slot*/) const { return ""; }
    virtual void onLeft (int /*slot*/) {}
    virtual void onRight(int /*slot*/) {}
};

// ---------------------------------------------------------------------------
// MFDPanel — transparent HUD frame with 6 side-buttons on each side.
//
// Delegates content rendering to the assigned MFDApp.
// Right button slot 5 is permanently reserved as a "MNU" button handled by
// the panel itself: it switches to/from the MFDMenu application.
//
// Call render() once per frame inside a valid ImGui frame.
// ---------------------------------------------------------------------------
struct MFDPanel {
    ImVec2    pos  {0.0f, 0.0f};   // screen-space top-left corner
    ImVec2    size {280.0f, 220.0f};
    MFDApp*   app      = nullptr;  // currently active non-menu app
    MFDMenu*  menuApp  = nullptr;  // the MFDMenu instance (null = no menu support)
    bool      isInMenu = false;    // true while showing the menu

    static constexpr float kBtnW   = 44.0f;
    static constexpr float kTitleH = 20.0f;

    // id must be a unique ImGui window ID string (e.g. "##MFD0").
    void render(const char* id);
};
