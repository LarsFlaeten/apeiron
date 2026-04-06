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
    // fullBleed() = true, so origin/size cover the full panel rect.
    const ImVec2 imgMin = origin;
    const ImVec2 imgMax = { origin.x + size.x, origin.y + size.y };

    if (m_texture != 0) {
        dl->AddImage(m_texture, imgMin, imgMax);
    } else {
        const ImU32 kGray = IM_COL32(80, 80, 80, 180);
        dl->AddRectFilled(imgMin, imgMax, IM_COL32(10, 10, 10, 200));
        const char* msg = "NO SIGNAL";
        ImVec2 msz = ImGui::CalcTextSize(msg);
        dl->AddText({imgMin.x + (size.x - msz.x) * 0.5f,
                     imgMin.y + (size.y - msz.y) * 0.5f}, kGray, msg);
    }
}
