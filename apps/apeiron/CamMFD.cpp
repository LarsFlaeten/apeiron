#include "CamMFD.h"

#include <glm/trigonometric.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
static const std::string kEmpty;

void CamMFD::setCamNodes(const std::vector<std::string>& nodes)
{
    m_camNodes = nodes;
    m_camIdx   = 0;
}

const std::string& CamMFD::activeCamNode() const
{
    if (m_camNodes.empty()) return kEmpty;
    return m_camNodes[static_cast<size_t>(m_camIdx)];
}

// ---------------------------------------------------------------------------
const char* CamMFD::leftLabel(int slot) const
{
    switch (slot) {
        case 0: return "CAM";
        case 1: return "RST";
        case 2: return "P+";
        case 3: return "P-";
        default: return "";
    }
}

void CamMFD::onLeft(int slot)
{
    if (slot == 0 && !m_camNodes.empty())
        m_camIdx = (m_camIdx + 1) % static_cast<int>(m_camNodes.size());
    else if (slot == 1)
        resetSlewZoom();
    else if (slot == 2)
        m_slewPitch = std::min( kSlewMax, m_slewPitch + slewStepDeg());
    else if (slot == 3)
        m_slewPitch = std::max(-kSlewMax, m_slewPitch - slewStepDeg());
}

// ---------------------------------------------------------------------------
const char* CamMFD::rightLabel(int slot) const
{
    switch (slot) {
        case 0: return "Z+";
        case 1: return "Z-";
        case 2: return "Y+";
        case 3: return "Y-";
        default: return "";
    }
}

void CamMFD::onRight(int slot)
{
    if (slot == 0)
        m_fovDeg = std::max( 5.0f, m_fovDeg / kZoomStep);
    else if (slot == 1)
        m_fovDeg = std::min(90.0f, m_fovDeg * kZoomStep);
    else if (slot == 2)
        m_slewYaw = std::min( kSlewMax, m_slewYaw + slewStepDeg());
    else if (slot == 3)
        m_slewYaw = std::max(-kSlewMax, m_slewYaw - slewStepDeg());
}

// ---------------------------------------------------------------------------
void CamMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    // fullBleed() = true — origin/size is the full panel rect.
    const ImVec2 imgMin = origin;
    const ImVec2 imgMax = { origin.x + size.x, origin.y + size.y };

    if (m_texture != 0) {
        // UV-crop the square offscreen texture to fill the panel without stretching.
        const float aspect = size.x / size.y;
        ImVec2 uv0{0.0f, 0.0f}, uv1{1.0f, 1.0f};
        if (aspect > 1.0f) {
            const float frac = 1.0f / aspect;
            uv0.y = 0.5f - frac * 0.5f;
            uv1.y = 0.5f + frac * 0.5f;
        } else if (aspect < 1.0f) {
            uv0.x = 0.5f - aspect * 0.5f;
            uv1.x = 0.5f + aspect * 0.5f;
        }
        dl->AddImage(m_texture, imgMin, imgMax, uv0, uv1);
    } else {
        const ImU32 kGray = IM_COL32(80, 80, 80, 180);
        dl->AddRectFilled(imgMin, imgMax, IM_COL32(10, 10, 10, 200));
        const char* msg = "NO SIGNAL";
        ImVec2 msz = ImGui::CalcTextSize(msg);
        dl->AddText({imgMin.x + (size.x - msz.x) * 0.5f,
                     imgMin.y + (size.y - msz.y) * 0.5f}, kGray, msg);
    }

    // ---- Overlays (drawn on top of the image) ----
    const float lh  = ImGui::GetTextLineHeight();
    const float pad = 4.0f;
    const ImU32 kDim  = IM_COL32(  0, 140,  50, 150);
    const ImU32 kPill = IM_COL32(  0,   0,   0, 160);
    char buf[64];

    // Content area: inset from full-bleed panel to avoid button columns.
    const ImVec2 cOrig { origin.x + MFDPanel::kBtnW,    origin.y + MFDPanel::kTitleH };
    const ImVec2 cSize { size.x   - 2.0f * MFDPanel::kBtnW,
                         size.y   - MFDPanel::kTitleH };

    auto pill = [&](float tx, float ty, ImVec2 tsz) {
        dl->AddRectFilled({ tx - pad, ty - 1.0f },
                          { tx + tsz.x + pad, ty + lh + 1.0f },
                          kPill, 3.0f);
    };

    // Camera name — bottom-centre.
    if (!m_camNodes.empty()) {
        const char* label = m_camNodes[static_cast<size_t>(m_camIdx)].c_str();
        ImVec2 tsz = ImGui::CalcTextSize(label);
        float tx = cOrig.x + (cSize.x - tsz.x) * 0.5f;
        float ty = imgMax.y - lh - pad * 2.0f;
        dl->AddRectFilled(
            { tx - pad,       ty - pad * 0.5f },
            { tx + tsz.x + pad, ty + lh + pad * 0.5f },
            IM_COL32(0, 0, 0, 140), 3.0f);
        dl->AddText({ tx, ty }, IM_COL32(0, 210, 75, 220), label);
    }

    // Zoom level — top-right of content area.
    // Expressed as a magnification relative to 70° baseline: x = tan(35°)/tan(fov/2).
    {
        const float zoomX = std::tan(glm::radians(35.0f))
                          / std::tan(glm::radians(m_fovDeg * 0.5f));
        std::snprintf(buf, sizeof(buf), "%.1fx", zoomX);
        ImVec2 zsz = ImGui::CalcTextSize(buf);
        float zx = cOrig.x + cSize.x - zsz.x - pad;
        float zy = cOrig.y + pad;
        pill(zx, zy, zsz);
        dl->AddText({ zx, zy }, kDim, buf);
    }

    // Slew offset — top-left of content area (only when non-zero).
    if (m_slewYaw != 0.0f || m_slewPitch != 0.0f) {
        std::snprintf(buf, sizeof(buf), "Y%+.0f P%+.0f", m_slewYaw, m_slewPitch);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        float tx = cOrig.x + pad;
        float ty = cOrig.y + pad;
        pill(tx, ty, tsz);
        dl->AddText({ tx, ty }, kDim, buf);
    }
}
