#include "MFD.h"
#include "MFDMenu.h"

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

    const ImU32 kGreen    = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDimLine  = IM_COL32(  0, 140,  50, 120);
    const ImU32 kBgFill   = IM_COL32(  0,  20,   8,  55);
    const ImU32 kMnuCol   = IM_COL32( 80, 180, 255, 220);  // distinct blue for MNU button

    const ImVec2 tl = pos;
    const ImVec2 br = { pos.x + size.x, pos.y + size.y };

    // Active app: menu when isInMenu, otherwise the normal app.
    MFDApp* activeApp = isInMenu ? static_cast<MFDApp*>(menuApp) : app;

    // Background fill.
    dl->AddRectFilled(tl, br, kBgFill, 3.0f);

    // Full-bleed apps render first (before chrome) so borders/buttons appear on top.
    if (activeApp && activeApp->fullBleed())
        activeApp->render(dl, tl, size);

    // Outer border.
    dl->AddRect(tl, br, kGreen, 3.0f, 0, 1.5f);

    // Title bar.
    const float titleBottom = pos.y + kTitleH;
    dl->AddLine({ tl.x, titleBottom }, { br.x, titleBottom }, kGreen, 1.0f);

    if (activeApp) {
        const char* title = activeApp->name();
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
    const int   kLastSlot = MFDApp::kSlots - 1;

    // Vertical dividers between buttons and content area.
    dl->AddLine({ tl.x + kBtnW, titleBottom }, { tl.x + kBtnW, br.y }, kGreen, 1.0f);
    dl->AddLine({ rightBtnX,    titleBottom }, { rightBtnX,    br.y }, kGreen, 1.0f);

    for (int s = 0; s < MFDApp::kSlots; ++s) {
        const float y0 = titleBottom + s * btnRowH;
        const float y1 = y0 + btnRowH;

        // Horizontal dividers inside button columns.
        if (s > 0) {
            dl->AddLine({ tl.x,      y0 }, { tl.x + kBtnW, y0 }, kDimLine, 0.5f);
            dl->AddLine({ rightBtnX, y0 }, { br.x,         y0 }, kDimLine, 0.5f);
        }

        // Hover / press highlight drawn before text so text sits on top.
        const ImU32 kHover  = IM_COL32(0, 160, 60, 40);
        const ImU32 kPress  = IM_COL32(0, 200, 70, 80);
        ImVec2 lR0 = {tl.x,      y0}, lR1 = {tl.x + kBtnW, y1};
        ImVec2 rR0 = {rightBtnX, y0}, rR1 = {br.x,          y1};
        bool lHov = ImGui::IsMouseHoveringRect(lR0, lR1);
        bool rHov = ImGui::IsMouseHoveringRect(rR0, rR1);
        bool lPrs = lHov && ImGui::IsMouseDown(ImGuiMouseButton_Left);
        bool rPrs = rHov && ImGui::IsMouseDown(ImGuiMouseButton_Left);
        if (lPrs)      dl->AddRectFilled(lR0, lR1, kPress);
        else if (lHov) dl->AddRectFilled(lR0, lR1, kHover);
        if (rPrs)      dl->AddRectFilled(rR0, rR1, kPress);
        else if (rHov) dl->AddRectFilled(rR0, rR1, kHover);

        // Left label — always delegate to active app.
        if (activeApp) {
            const char* ll = activeApp->leftLabel(s);
            if (ll && *ll) {
                ImVec2 lsz = ImGui::CalcTextSize(ll);
                dl->AddText(
                    { tl.x + (kBtnW - lsz.x) * 0.5f,
                      y0   + (btnRowH - lsz.y) * 0.5f },
                    kGreen, ll);
            }
        }

        // Right label — slot kLastSlot is permanently "MNU"; others delegate to app.
        const char* rl    = "";
        ImU32       rlCol = kGreen;
        if (s == kLastSlot && menuApp) {
            rl    = "MNU";
            rlCol = isInMenu ? kGreen : kMnuCol;
        } else if (activeApp && !isInMenu) {
            rl = activeApp->rightLabel(s);
        }
        if (rl && *rl) {
            ImVec2 rsz = ImGui::CalcTextSize(rl);
            dl->AddText(
                { rightBtnX + (kBtnW - rsz.x) * 0.5f,
                  y0        + (btnRowH - rsz.y) * 0.5f },
                rlCol, rl);
        }

        // Invisible buttons for click detection.
        std::string lid = std::string("##L") + std::to_string(s) + id;
        std::string rid = std::string("##R") + std::to_string(s) + id;

        ImGui::SetCursorScreenPos({ tl.x, y0 });
        if (ImGui::InvisibleButton(lid.c_str(), { kBtnW, btnRowH }) && activeApp && !isInMenu)
            activeApp->onLeft(s);

        ImGui::SetCursorScreenPos({ rightBtnX, y0 });
        if (ImGui::InvisibleButton(rid.c_str(), { kBtnW, btnRowH })) {
            if (s == kLastSlot && menuApp) {
                // MNU: toggle menu.
                isInMenu = !isInMenu;
            } else if (activeApp && !isInMenu) {
                activeApp->onRight(s);
            }
        }
    }

    // Content area — delegate to active app (skipped for full-bleed apps, already drawn).
    if (activeApp && !activeApp->fullBleed()) {
        const ImVec2 contentOrigin = { tl.x + kBtnW,   titleBottom };
        const ImVec2 contentSize   = { size.x - 2.0f * kBtnW, contentH };
        activeApp->render(dl, contentOrigin, contentSize);
    }

    // If in menu mode, check whether the user selected an app this frame.
    if (isInMenu && menuApp) {
        if (MFDApp* sel = menuApp->consumeSelection()) {
            app      = sel;
            isInMenu = false;
        }
    }

    ImGui::End();
}
