#pragma once

#include "MFD.h"

#include <imgui.h>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// CamMFD — displays a live camera view from a spacecraft camera node.
//
// The actual rendering to texture is handled by OffscreenCam in the main
// render loop (before the HDR pass).  CamMFD receives the finished texture
// via setTexture() each frame and displays it via ImDrawList::AddImage().
//
// Button layout:
//   Left  0  "CAM" — cycle camera nodes
//   Left  1  "RST" — reset slew and zoom to defaults
//   Left  2  "P+"  — pitch view up
//   Left  3  "P-"  — pitch view down
//   Right 0  "Z+"  — zoom in  (FOV ÷ √2 per press)
//   Right 1  "Z-"  — zoom out (FOV × √2 per press)
//   Right 2  "Y+"  — yaw view right
//   Right 3  "Y-"  — yaw view left
//
// Slew step scales with FOV so control is finer when zoomed in.
// ---------------------------------------------------------------------------
class CamMFD : public MFDApp {
public:
    const char* name() const override { return "CAM"; }

    // Register the cam_* node list extracted from the player GLB.
    void setCamNodes(const std::vector<std::string>& nodes);

    // Name of the currently selected camera node (used by main.cpp to set
    // up the offscreen VP matrix).  Returns empty string if no nodes.
    const std::string& activeCamNode() const;

    // Feed the rendered texture for this frame (call before render()).
    void setTexture(ImTextureID tex) { m_texture = tex; }
    bool hasTexture() const          { return m_texture != 0; }

    // Slew offsets (degrees) applied on top of the camera node's orientation.
    float slewYaw()   const { return m_slewYaw;   }
    float slewPitch() const { return m_slewPitch;  }

    // Current field of view (degrees).
    float fovDeg() const { return m_fovDeg; }

    bool fullBleed() const override { return true; }
    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    void        onLeft    (int slot) override;
    const char* rightLabel(int slot) const override;
    void        onRight   (int slot) override;

private:
    void resetSlewZoom() { m_slewYaw = 0.0f; m_slewPitch = 0.0f; m_fovDeg = kDefaultFov; }

    // Slew step in degrees at the default FOV; scales proportionally so
    // pressing a button moves fewer degrees when zoomed in (narrower FOV).
    float slewStepDeg() const { return kBaseSlewStep * (m_fovDeg / kDefaultFov); }

    static constexpr float kDefaultFov   = 70.0f;
    static constexpr float kZoomStep     = 1.4142f;  // ~1 stop per press, 5°–90°
    static constexpr float kBaseSlewStep = 5.0f;      // degrees per press at 70° FOV
    static constexpr float kSlewMax      = 90.0f;     // max slew magnitude per axis

    std::vector<std::string> m_camNodes;
    int                      m_camIdx   = 0;
    ImTextureID              m_texture  = 0;

    float m_slewYaw   = 0.0f;
    float m_slewPitch = 0.0f;
    float m_fovDeg    = kDefaultFov;
};
