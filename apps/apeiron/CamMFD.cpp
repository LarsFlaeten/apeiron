#include "CamMFD.h"

#include <algorithm>
#include <cmath>

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
    if (slot == 0) return "CAM";
    return "";
}

void CamMFD::onLeft(int slot)
{
    if (slot == 0 && !m_camNodes.empty())
        m_camIdx = (m_camIdx + 1) % static_cast<int>(m_camNodes.size());
}

// ---------------------------------------------------------------------------
void CamMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    // fullBleed() = true — origin/size is the full panel rect.
    const ImVec2 imgMin = origin;
    const ImVec2 imgMax = { origin.x + size.x, origin.y + size.y };

    if (m_texture != 0) {
        // UV-crop the square offscreen texture to fill the panel without stretching.
        // For a panel wider than it is tall (aspect > 1): fill width, crop height.
        // For a panel taller than it is wide (aspect < 1): fill height, crop width.
        const float aspect = size.x / size.y;   // panel aspect (w/h)
        ImVec2 uv0{0.0f, 0.0f}, uv1{1.0f, 1.0f};
        if (aspect > 1.0f) {
            // Crop top/bottom: show only a 1/aspect fraction of texture height, centred.
            const float frac = 1.0f / aspect;
            uv0.y = 0.5f - frac * 0.5f;
            uv1.y = 0.5f + frac * 0.5f;
        } else if (aspect < 1.0f) {
            // Crop left/right: show only an aspect fraction of texture width, centred.
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

    // Camera name overlay — bottom-centre, semi-transparent backdrop.
    if (!m_camNodes.empty()) {
        const char*  label = m_camNodes[static_cast<size_t>(m_camIdx)].c_str();
        const float  pad   = 4.0f;
        const float  lh    = ImGui::GetTextLineHeight();
        ImVec2 tsz = ImGui::CalcTextSize(label);
        const float tx = imgMin.x + (size.x - tsz.x) * 0.5f;
        const float ty = imgMax.y - lh - pad * 2.0f;
        // Dark pill behind text.
        dl->AddRectFilled(
            { tx - pad,       ty - pad * 0.5f },
            { tx + tsz.x + pad, ty + lh + pad * 0.5f },
            IM_COL32(0, 0, 0, 140), 3.0f);
        dl->AddText({ tx, ty }, IM_COL32(0, 210, 75, 220), label);
    }
}
