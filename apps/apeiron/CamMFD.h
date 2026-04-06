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
// Left button slot 0 ("CAM") cycles through available camera nodes.
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

    bool fullBleed() const override { return true; }
    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel(int slot) const override;
    void        onLeft   (int slot) override;

private:
    std::vector<std::string> m_camNodes;
    int                      m_camIdx  = 0;
    ImTextureID              m_texture = 0;
};
