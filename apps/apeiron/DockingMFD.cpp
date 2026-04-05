#include "DockingMFD.h"

void DockingMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kDim = IM_COL32(0, 140, 50, 150);
    const char* msg  = "-- DOCKING MFD --";
    ImVec2 tsz = ImGui::CalcTextSize(msg);
    dl->AddText(
        { origin.x + (size.x - tsz.x) * 0.5f,
          origin.y + (size.y - tsz.y) * 0.5f },
        kDim, msg);
}
