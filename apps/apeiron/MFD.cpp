#include "MFD.h"

#include <string>

void MFDPanel::render(const char* id)
{
    ImGui::SetNextWindowPos (pos,  ImGuiCond_Always);
    ImGui::SetNextWindowSize(size, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::Begin(id, nullptr,
        ImGuiWindowFlags_NoDecoration       |
        ImGuiWindowFlags_NoMove             |
        ImGuiWindowFlags_NoSavedSettings    |
        ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoNav              |
        ImGuiWindowFlags_NoScrollWithMouse);

    ImDrawList* dl = ImGui::GetWindowDrawList();

    const ImU32 kGreen    = IM_COL32(  0, 220,  80, 255);
    const ImU32 kDimLine  = IM_COL32(  0, 140,  50, 160);
    const ImU32 kBgFill   = IM_COL32(  0,  25,  10, 110);

    const ImVec2 tl = pos;
    const ImVec2 br = { pos.x + size.x, pos.y + size.y };

    // Background fill (very faint green tint for readability).
    dl->AddRectFilled(tl, br, kBgFill, 3.0f);
    // Outer border.
    dl->AddRect(tl, br, kGreen, 3.0f, 0, 1.5f);

    // Title bar.
    const float titleBottom = pos.y + kTitleH;
    dl->AddLine({ tl.x, titleBottom }, { br.x, titleBottom }, kGreen, 1.0f);

    if (app) {
        const char* title = app->name();
        ImVec2 tsz = ImGui::CalcTextSize(title);
        dl->AddText(
            { tl.x + (size.x - tsz.x) * 0.5f,
              tl.y + (kTitleH - tsz.y) * 0.5f },
            kGreen, title);
    }

    // Side buttons — 6 rows equally sharing the content height.
    const float contentH  = size.y - kTitleH;
    const float btnRowH   = contentH / static_cast<float>(MFDApp::kSlots);
    const float rightBtnX = pos.x + size.x - kBtnW;

    // Vertical dividers between buttons and content area.
    dl->AddLine({ tl.x + kBtnW, titleBottom }, { tl.x + kBtnW, br.y }, kGreen, 1.0f);
    dl->AddLine({ rightBtnX,    titleBottom }, { rightBtnX,    br.y }, kGreen, 1.0f);

    for (int s = 0; s < MFDApp::kSlots; ++s) {
        const float y0 = titleBottom + s * btnRowH;
        const float y1 = y0 + btnRowH;

        // Horizontal dividers inside button columns.
        if (s > 0) {
            dl->AddLine({ tl.x,     y0 }, { tl.x + kBtnW, y0 }, kDimLine, 0.5f);
            dl->AddLine({ rightBtnX, y0 }, { br.x,         y0 }, kDimLine, 0.5f);
        }

        if (app) {
            // Left label — centred inside the button cell.
            const char* ll = app->leftLabel(s);
            if (ll && *ll) {
                ImVec2 lsz = ImGui::CalcTextSize(ll);
                dl->AddText(
                    { tl.x + (kBtnW - lsz.x) * 0.5f,
                      y0   + (btnRowH - lsz.y) * 0.5f },
                    kGreen, ll);
            }

            // Right label.
            const char* rl = app->rightLabel(s);
            if (rl && *rl) {
                ImVec2 rsz = ImGui::CalcTextSize(rl);
                dl->AddText(
                    { rightBtnX + (kBtnW - rsz.x) * 0.5f,
                      y0        + (btnRowH - rsz.y) * 0.5f },
                    kGreen, rl);
            }
        }

        // Invisible buttons for click detection.
        // IDs are unique per slot + per panel (id suffix).
        std::string lid = std::string("##L") + std::to_string(s) + id;
        std::string rid = std::string("##R") + std::to_string(s) + id;

        ImGui::SetCursorScreenPos({ tl.x, y0 });
        if (ImGui::InvisibleButton(lid.c_str(), { kBtnW, btnRowH }) && app)
            app->onLeft(s);

        ImGui::SetCursorScreenPos({ rightBtnX, y0 });
        if (ImGui::InvisibleButton(rid.c_str(), { kBtnW, btnRowH }) && app)
            app->onRight(s);
    }

    // Content area — delegate to app.
    if (app) {
        const ImVec2 contentOrigin = { tl.x + kBtnW,   titleBottom };
        const ImVec2 contentSize   = { size.x - 2.0f * kBtnW, contentH };
        app->render(dl, contentOrigin, contentSize);
    }

    ImGui::End();
}
