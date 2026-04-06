#pragma once

#include "MFD.h"
#include "Spacecraft.h"
#include "NavState.h"
#include "DockingConstraint.h"
#include "apeiron/render/GltfModel.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// DockingMFD — docking approach guidance display.
//
// Layout (content area, top → bottom):
//   cam label            — name of the active camera node
//   RNG / VEL row        — range to target port + axial closure rate
//   lateral indicator    — 2D cross-hair showing target port's offset in the
//                          player port's YZ plane (translation misalignment)
//   P / Y / R row        — approach axis pitch+yaw error + roll error
//
// The indicator is a "slave" to NavConsole: it reads dockTgtIdx / dockPortIdx
// from NavState and only shows guidance when a target port is selected.
//
// Camera: setCamNodes() provides available cam nodes from the Orion GLB.
// Left-button slot 0 ("CAM") cycles through them.  When a port is selected,
// the MFD auto-selects a cam whose name contains "dock" (if available).
// Call preferredCamNode() each frame; main.cpp uses it to override the scene
// camera when DockingMFD is the active full-screen app.
// ---------------------------------------------------------------------------
class DockingMFD : public MFDApp {
public:
    using DockPort = apeiron::render::GltfModel::DockingPort;

    const char* name() const override { return "DOCK"; }

    // Register cam nodes extracted from the player GLB (call once after load).
    void setCamNodes(const std::vector<std::string>& nodes);

    // Returns the camera node this MFD currently wants to use.
    // Empty string = no preference (main.cpp keeps its own navCamIdx).
    const std::string& preferredCamNode() const;

    // Compute docking geometry for this frame.  Must be called before render().
    void update(const Spacecraft&                                ship,
                const std::vector<std::vector<DockPort>>&        scPorts,
                size_t                                           playerIdx,
                const NavState&                                  nav,
                const std::vector<std::unique_ptr<Spacecraft>>&  spacecraft);

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    // Feed the current capture phase from DockingConstraint so the MFD can show it.
    enum class CapturePhase { None, Unarmed, Armed, SoftCapture, HardCapture };
    void setCapturePhase(CapturePhase p) { m_capturePhase = p; }

    // Feed the offscreen camera texture each frame (0 = no texture, normal layout).
    void setTexture(ImTextureID tex) { m_texture = tex; }

    // Full-bleed only when a camera texture is available.
    bool fullBleed() const override { return m_texture != 0; }

    // Provide a pointer to the constraint so the ARM button can call activate().
    void setConstraint(DockingConstraint* c) { m_constraint = c; }

    // Feed capture thresholds for colour-coding the range / velocity readouts.
    void setCaptureThresholds(float maxRangeM, float maxClosureMs, float maxAttErrDeg) {
        m_threshRangeM    = maxRangeM;
        m_threshClosureMs = maxClosureMs;
        m_threshAttErrDeg = maxAttErrDeg;
    }

    // Camera FOV for the offscreen docking cam (main.cpp reads this each frame).
    float fovDeg() const { return m_fovDeg; }

    const char* leftLabel (int slot) const override;
    void        onLeft    (int slot) override;
    const char* rightLabel(int slot) const override;
    void        onRight   (int slot) override;

private:
    // Try to auto-select a cam for the given port label.
    void autoSelectCam(const std::string& portLabel);

    // ---- camera ----
    std::vector<std::string> m_camNodes;
    int                      m_camIdx     = 0;
    int                      m_lastPortIdx = -2;  // track port changes for auto-select

    // ---- docking geometry (computed by update()) ----
    bool   m_hasTarget   = false;  // a docking target spacecraft is selected
    bool   m_hasPort     = false;  // a specific target port is selected
    float  m_rangeM      = 0.0f;  // distance between the two ports (m)
    float  m_closureMs   = 0.0f;  // axial closure rate (m/s, +ve = approaching)
    float  m_latY        = 0.0f;  // lateral offset in port Y axis (m)
    float  m_latZ        = 0.0f;  // lateral offset in port Z axis (m)
    float  m_pitchErrDeg = 0.0f;  // approach axis pitch mismatch
    float  m_yawErrDeg   = 0.0f;  // approach axis yaw mismatch
    float  m_rollErrDeg  = 0.0f;  // port roll mismatch
    float  m_fullScaleM  = 50.0f; // half-width of indicator in metres (dynamic)

    CapturePhase       m_capturePhase     = CapturePhase::None;
    DockingConstraint* m_constraint       = nullptr;
    ImTextureID        m_texture          = 0;
    float              m_fovDeg           = 70.0f;  // camera zoom (narrow = zoomed in)
    float              m_threshRangeM     = 0.15f;
    float              m_threshClosureMs  = 0.03f;
    float              m_threshAttErrDeg  = 5.0f;
};
