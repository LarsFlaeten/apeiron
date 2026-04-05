#include "MFDMenu.h"

void MFDMenu::addApp(MFDApp* app, const char* label)
{
    m_entries.push_back({app, label});
}

MFDApp* MFDMenu::consumeSelection()
{
    MFDApp* sel = m_selected;
    m_selected  = nullptr;
    return sel;
}

void MFDMenu::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen   = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDimBox  = IM_COL32(  0, 140,  50, 120);
    const ImU32 kHoverBg = IM_COL32(  0, 210,  75,  45);

    constexpr int   kCols = 2;
    constexpr float kPad  = 8.0f;
    const float tileW = (size.x - kPad * (kCols + 1)) / static_cast<float>(kCols);
    const float tileH = tileW * 0.55f;

    const ImVec2 mousePos = ImGui::GetMousePos();
    const bool   clicked  = ImGui::IsMouseClicked(ImGuiMouseButton_Left);

    for (int i = 0; i < static_cast<int>(m_entries.size()); ++i) {
        const int col = i % kCols;
        const int row = i / kCols;

        const ImVec2 tl {
            origin.x + kPad + static_cast<float>(col) * (tileW + kPad),
            origin.y + kPad + static_cast<float>(row) * (tileH + kPad)
        };
        const ImVec2 br { tl.x + tileW, tl.y + tileH };

        const bool hovered = mousePos.x >= tl.x && mousePos.x < br.x &&
                             mousePos.y >= tl.y && mousePos.y < br.y;

        if (hovered) {
            dl->AddRectFilled(tl, br, kHoverBg, 4.0f);
            if (clicked)
                m_selected = m_entries[static_cast<size_t>(i)].app;
        }

        dl->AddRect(tl, br, hovered ? kGreen : kDimBox, 4.0f, 0, 1.5f);

        // Centred label.
        const char* lbl = m_entries[static_cast<size_t>(i)].label;
        ImVec2 tsz = ImGui::CalcTextSize(lbl);
        dl->AddText(
            { tl.x + (tileW - tsz.x) * 0.5f,
              tl.y + (tileH - tsz.y) * 0.5f },
            kGreen, lbl);
    }
}
