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
    const ImU32 kDim  = IM_COL32(  0, 140,  50, 150);
    const ImU32 kGray = IM_COL32( 80,  80,  80, 180);
    const float lh    = ImGui::GetTextLineHeight();
    const float pad   = 4.0f;

    // ---- Camera node label (top centre) ----
    if (!m_camNodes.empty()) {
        const char* label = m_camNodes[static_cast<size_t>(m_camIdx)].c_str();
        ImVec2 tsz = ImGui::CalcTextSize(label);
        dl->AddText({origin.x + (size.x - tsz.x) * 0.5f, origin.y + pad}, kDim, label);
    }

    // ---- Image area: square, centred below the label ----
    const float topY   = origin.y + lh + pad * 2.0f;
    const float availW = size.x;
    const float availH = size.y - (topY - origin.y);
    const float side   = std::min(availW, availH);
    const float x0     = origin.x + (availW - side) * 0.5f;
    const float y0     = topY     + (availH - side) * 0.5f;

    const ImVec2 imgMin{x0, y0};
    const ImVec2 imgMax{x0 + side, y0 + side};

    if (m_texture != 0) {
        dl->AddImage(m_texture, imgMin, imgMax);
    } else {
        dl->AddRectFilled(imgMin, imgMax, IM_COL32(10, 10, 10, 200));
        dl->AddRect      (imgMin, imgMax, kGray);
        const char* msg = "NO SIGNAL";
        ImVec2 msz = ImGui::CalcTextSize(msg);
        dl->AddText({imgMin.x + (side - msz.x) * 0.5f,
                     imgMin.y + (side - msz.y) * 0.5f}, kGray, msg);
    }
}
