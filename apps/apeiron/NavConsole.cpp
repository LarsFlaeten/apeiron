#include "NavConsole.h"

#include <imgui.h>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <numbers>

// ---------------------------------------------------------------------------
void NavConsole::update(const Spacecraft&  ship,
                        const glm::dvec3&  shipForce,
                        double             shipMass,
                        float              frameDt)
{
    glm::dquat att    = ship.attitude();
    glm::dvec3 w_in   = ship.angularVelocity();
    glm::dvec3 w_bod  = glm::conjugate(att) * w_in;

    // Angular acceleration via finite difference.
    glm::dvec3 alpha_bod(0.0);
    if (m_prevDt > 1e-6f) {
        glm::dvec3 prevW_bod = glm::conjugate(att) * m_prevAngVelInertial;
        alpha_bod = (w_bod - prevW_bod) / static_cast<double>(m_prevDt);
    }
    m_prevAngVelInertial = w_in;
    m_prevDt             = frameDt;

    // EMA (τ = 0.15 s).
    constexpr double kTau = 0.15;
    double emaAlpha = static_cast<double>(frameDt) / (static_cast<double>(frameDt) + kTau);

    constexpr double kR2D = 180.0 / std::numbers::pi;
    glm::dvec3 w_degs  = w_bod     * kR2D;
    glm::dvec3 al_degs = alpha_bod * kR2D;
    m_emaAngVel_degs += emaAlpha * (w_degs  - m_emaAngVel_degs);
    m_emaAngAcc_degs += emaAlpha * (al_degs - m_emaAngAcc_degs);

    // Felt linear acceleration (F/m, body frame, gravity excluded).
    constexpr double kG = 9.80665;
    m_linAcc_g = shipForce / (shipMass * kG);

    // Prograde error (angle between nose +X and velocity vector).
    glm::dvec3 vel    = ship.velocity();
    double     vMag   = glm::length(vel);
    m_progradeErrDeg  = 0.0;
    if (vMag > 1e-6) {
        glm::dvec3 nose = att * glm::dvec3(1.0, 0.0, 0.0);
        double c = glm::clamp(glm::dot(nose, vel / vMag), -1.0, 1.0);
        m_progradeErrDeg = std::acos(c) * 180.0 / std::numbers::pi;
    }

    // Vertical speed (radial component of velocity, km/s).
    m_vsKms = 0.0;
    if (glm::length(ship.position()) > 1e-6) {
        glm::dvec3 rHat = glm::normalize(ship.position());
        m_vsKms = glm::dot(vel, rHat);
    }

    m_shipMass = shipMass;
}

// ---------------------------------------------------------------------------
void NavConsole::render(float                                           posX,
                        float                                           posY,
                        float                                           width,
                        NavState&                                       nav,
                        spacecraft::Autopilot&                          autopilot,
                        std::vector<std::unique_ptr<Spacecraft>>&        spacecraft,
                        size_t                                          playerIdx,
                        const std::vector<std::string>&                 scNames,
                        const std::vector<std::vector<DockPort>>&       scPorts,
                        bool                                            mainEngineOn,
                        const glm::dvec3&                               mccDv)
{
    const float  consW  = width;
    const ImU32  kGrn   = IM_COL32(  0, 210,  75, 210);
    const ImU32  kDim   = IM_COL32(  0, 160,  55, 150);

    constexpr auto kFlags =
        ImGuiWindowFlags_NoDecoration     |
        ImGuiWindowFlags_NoSavedSettings  |
        ImGuiWindowFlags_NoFocusOnAppearing|
        ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::SetNextWindowBgAlpha(0.30f);
    ImGui::SetNextWindowPos(ImVec2(posX, posY), ImGuiCond_Always, ImVec2(0.0f, 1.0f));
    ImGui::SetNextWindowSize(ImVec2(consW, 0.0f));
    ImGui::Begin("##NavConsole", nullptr, kFlags);

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kGrn));

    // ---- Title + MODE button ----
    {
        ImGui::SetCursorPosX((consW - ImGui::CalcTextSize("-- NAV CONSOLE --").x) * 0.5f);
        ImGui::TextColored({0.0f, 0.82f, 0.30f, 0.8f}, "-- NAV CONSOLE --");
        ImGui::SameLine();
        if (nav.navMode == NavMode::Orbit)
            ImGui::TextColored({0.0f, 0.82f, 0.30f, 1.0f}, "[ORBIT]");
        else if (nav.navMode == NavMode::Docking)
            ImGui::TextColored({0.2f, 0.8f, 1.0f, 1.0f}, "[DOCK]");
        else
            ImGui::TextColored({1.0f, 0.55f, 0.0f, 1.0f}, "[MCC]");
        ImGui::SameLine();
        if (ImGui::SmallButton("MODE")) {
            // Cycle: Orbit → Docking → Mcc → Orbit
            if      (nav.navMode == NavMode::Orbit)   nav.navMode = NavMode::Docking;
            else if (nav.navMode == NavMode::Docking)  nav.navMode = NavMode::Mcc;
            else                                       nav.navMode = NavMode::Orbit;
            autopilot.mode = spacecraft::AutopilotMode::Off;
        }
    }

    // ---- TRANS: FINE / AGGR selector ----
    {
        const bool isFine = (nav.transMode == TransMode::Fine);
        ImGui::TextColored({0.0f, 0.82f, 0.30f, 0.6f}, "TRANS:");
        ImGui::SameLine();
        if (isFine)  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f,  0.45f, 0.1f,  1.0f));
        if (ImGui::SmallButton("FINE"))  nav.transMode = TransMode::Fine;
        if (isFine)  ImGui::PopStyleColor();
        ImGui::SameLine();
        if (!isFine) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.25f, 0.05f, 1.0f));
        if (ImGui::SmallButton("AGGR"))  nav.transMode = TransMode::Aggressive;
        if (!isFine) ImGui::PopStyleColor();
    }
    ImGui::Separator();

    // Two-column row helper.
    auto rowf = [&](const char* lbl, const char* fmt, ...) {
        char buf[64]; va_list ap; va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kDim));
        ImGui::Text("%s", lbl);
        ImGui::PopStyleColor();
        ImGui::SameLine(consW * 0.42f);
        ImGui::Text("%s", buf);
    };

    // ---- Rotational rates ----
    rowf("ω Pitch", "%+7.2f °/s",  m_emaAngVel_degs.y);
    rowf("ω Yaw  ", "%+7.2f °/s",  m_emaAngVel_degs.z);
    rowf("ω Roll ", "%+7.2f °/s",  m_emaAngVel_degs.x);
    ImGui::Separator();

    // ---- Angular accelerations ----
    rowf("α Pitch", "%+7.2f °/s²", m_emaAngAcc_degs.y);
    rowf("α Yaw  ", "%+7.2f °/s²", m_emaAngAcc_degs.z);
    rowf("α Roll ", "%+7.2f °/s²", m_emaAngAcc_degs.x);
    ImGui::Separator();

    // ---- Felt linear acceleration ----
    rowf("Acc X  ", "%+7.3f g", m_linAcc_g.x);
    rowf("Acc Y  ", "%+7.3f g", m_linAcc_g.y);
    rowf("Acc Z  ", "%+7.3f g", m_linAcc_g.z);
    ImGui::Separator();

    // ---- Orbit-mode flight data ----
    if (nav.navMode == NavMode::Orbit) {
        rowf("Pro∠   ", "%7.1f °",      m_progradeErrDeg);
        rowf("Vspd   ", "%+7.3f km/s",  m_vsKms);
        ImGui::Separator();
    }

    // ---- MCC mode: course correction data ----
    if (nav.navMode == NavMode::Mcc) {
        const double mccMagMs = glm::length(mccDv) * 1000.0;
        const ImU32  kOrange  = IM_COL32(255, 140, 0, 230);
        if (mccMagMs > 1e-6) {
            ImGui::PushStyleColor(ImGuiCol_Text,
                ImGui::ColorConvertU32ToFloat4(kOrange));
            ImGui::Text("MCC  %.3f m/s", mccMagMs);
            ImGui::PopStyleColor();
            ImGui::PushStyleColor(ImGuiCol_Text,
                ImGui::ColorConvertU32ToFloat4(IM_COL32(255,140,0,160)));
            ImGui::Text("  X %+.4f km/s", mccDv.x);
            ImGui::Text("  Y %+.4f km/s", mccDv.y);
            ImGui::Text("  Z %+.4f km/s", mccDv.z);
            ImGui::PopStyleColor();
        } else {
            ImGui::TextColored({0.0f, 0.82f, 0.30f, 0.6f}, "MCC  on target");
        }
        ImGui::Separator();
    }

    // ---- Main engine indicator ----
    if (mainEngineOn)
        ImGui::TextColored({1.0f, 0.50f, 0.10f, 1.0f}, "  *** MAIN ENGINE ***");
    else
        ImGui::TextColored({0.0f, 0.50f, 0.20f, 0.5f}, "  main engine off");
    ImGui::Separator();

    // ---- Docking mode: TGT selector + proximity data ----
    if (nav.navMode == NavMode::Docking) {
        // Spacecraft-level target selector
        const char* tgtName = (nav.dockTgtIdx < 0)
            ? "NONE"
            : (nav.dockTgtIdx < static_cast<int>(scNames.size())
                ? scNames[nav.dockTgtIdx].c_str() : "?");
        ImGui::TextColored({0.2f, 0.8f, 1.0f, 0.8f}, "TGT:");
        ImGui::SameLine();
        ImGui::TextColored({0.2f, 0.8f, 1.0f, 1.0f}, "%s", tgtName);
        ImGui::SameLine();
        if (ImGui::SmallButton("[SEL]")) {
            ++nav.dockTgtIdx;
            if (nav.dockTgtIdx >= static_cast<int>(spacecraft.size()))
                nav.dockTgtIdx = -1;
            if (nav.dockTgtIdx == static_cast<int>(playerIdx))
                ++nav.dockTgtIdx;  // skip player
            if (nav.dockTgtIdx >= static_cast<int>(spacecraft.size()))
                nav.dockTgtIdx = -1;
            nav.dockPortIdx = -1;  // reset port when switching spacecraft
        }

        if (nav.dockTgtIdx >= 0) {
            // Port sub-selector — only shown when compatible ports are available
            if (nav.compatiblePortCount > 0) {
                const auto& tgtPorts = (nav.dockTgtIdx < static_cast<int>(scPorts.size()))
                                       ? scPorts[static_cast<size_t>(nav.dockTgtIdx)]
                                       : std::vector<DockPort>{};

                const char* portLabel = (nav.dockPortIdx < 0)
                    ? "CoM"
                    : (nav.dockPortIdx < static_cast<int>(tgtPorts.size())
                        ? tgtPorts[nav.dockPortIdx].label.c_str() : "?");

                ImGui::TextColored({0.2f, 0.8f, 1.0f, 0.6f}, "PORT:");
                ImGui::SameLine();
                ImGui::TextColored({0.2f, 0.8f, 1.0f, 1.0f}, "%s", portLabel);
                ImGui::SameLine();
                if (ImGui::SmallButton("[PORT]")) {
                    // Cycle: CoM(-1) → compatible port 0 → compatible port 1 → ... → CoM
                    // Only cycle through ports that match active/passive
                    bool playerHasActive  = false;
                    bool playerHasPassive = false;
                    for (auto& pp : (playerIdx < scPorts.size() ? scPorts[playerIdx]
                                                                 : std::vector<DockPort>{})) {
                        if (pp.active)  playerHasActive  = true;
                        else            playerHasPassive = true;
                    }

                    // Build compatible index list
                    std::vector<int> compatIdxs;
                    for (int i = 0; i < static_cast<int>(tgtPorts.size()); ++i) {
                        auto& tp = tgtPorts[i];
                        if ((tp.active && playerHasPassive) || (!tp.active && playerHasActive))
                            compatIdxs.push_back(i);
                    }

                    if (!compatIdxs.empty()) {
                        // Find current position in the compatible list
                        int cur = -1;
                        for (int i = 0; i < static_cast<int>(compatIdxs.size()); ++i)
                            if (compatIdxs[i] == nav.dockPortIdx) { cur = i; break; }
                        // Advance; wrap -1 → first compatible, last → -1 (CoM)
                        if (cur + 1 < static_cast<int>(compatIdxs.size()))
                            nav.dockPortIdx = compatIdxs[cur + 1];
                        else
                            nav.dockPortIdx = -1;
                    }
                }
            }

            // Distance / relative velocity
            auto& tgt    = *spacecraft[static_cast<size_t>(nav.dockTgtIdx)];
            auto& player = *spacecraft[playerIdx];

            // When a specific target port is selected, measure port-to-port distance.
            double distKm = glm::length(player.position() - tgt.position());
            const auto& tPorts = (nav.dockTgtIdx < static_cast<int>(scPorts.size()))
                                 ? scPorts[static_cast<size_t>(nav.dockTgtIdx)]
                                 : std::vector<DockPort>{};
            const auto& pPorts = (playerIdx < scPorts.size()) ? scPorts[playerIdx]
                                                               : std::vector<DockPort>{};
            if (nav.dockPortIdx >= 0 &&
                nav.dockPortIdx < static_cast<int>(tPorts.size()) &&
                !pPorts.empty()) {
                const auto& tp = tPorts[static_cast<size_t>(nav.dockPortIdx)];
                const auto& pp = pPorts[0];
                glm::dvec3 tPortPos = tgt.position()    + tgt.attitude()    * glm::dvec3(tp.posM) * 1e-3;
                glm::dvec3 pPortPos = player.position() + player.attitude() * glm::dvec3(pp.posM) * 1e-3;
                distKm = glm::length(tPortPos - pPortPos);
            }

            double distM  = distKm * 1000.0;
            double relVMs = glm::length(player.velocity() - tgt.velocity()) * 1000.0;
            if (distM < 1000.0)
                rowf("DIST   ", "%7.1f m",   distM);
            else
                rowf("DIST   ", "%7.3f km",  distKm);
            rowf("VR     ", "%7.3f m/s", relVMs);

            // Time and distance to null relative velocity, assuming NullV autopilot
            // engages now with current RCS authority in the relative-velocity direction.
            // a = F/m where F is the authority the autopilot would command.
            {
                const double aMs2 = autopilot.nullVAuthN / m_shipMass;  // m/s²
                if (aMs2 > 1e-9 && relVMs > 1e-3) {
                    double tNull = relVMs / aMs2;                        // s
                    double dNull = (relVMs * relVMs) / (2.0 * aMs2);    // m
                    if (tNull < 60.0)
                        rowf("T-NV   ", "%7.1f s",  tNull);
                    else
                        rowf("T-NV   ", "%7.1f min", tNull / 60.0);
                    if (dNull < 1000.0)
                        rowf("D-NV   ", "%7.1f m",  dNull);
                    else
                        rowf("D-NV   ", "%7.3f km", dNull / 1000.0);
                } else {
                    rowf("T-NV   ", "  --");
                    rowf("D-NV   ", "  --");
                }
            }
        }
        ImGui::Separator();
    }

    // ---- Autopilot status + buttons ----
    using M = spacecraft::AutopilotMode;
    auto apMode = autopilot.mode;

    const char* apLabel = "off";
    if      (apMode == M::Killrot)     apLabel = "KILLROT";
    else if (apMode == M::Prograde)    apLabel = "PROGRADE";
    else if (apMode == M::Retrograde)  apLabel = "RETROGRADE";
    else if (apMode == M::NormalPlus)  apLabel = "NORMAL+";
    else if (apMode == M::NormalMinus) apLabel = "NORMAL-";
    else if (apMode == M::NullV)       apLabel = "NULL V";
    else if (apMode == M::RelVelPlus)  apLabel = "+V";
    else if (apMode == M::RelVelMinus) apLabel = "-V";
    else if (apMode == M::MccPlus)     apLabel = "MCC+";
    ImGui::TextColored({0.0f, 0.82f, 0.30f, 0.6f}, "AP:");
    ImGui::SameLine();
    ImGui::TextColored(apMode != M::Off
        ? ImVec4{0.2f, 1.0f, 0.4f, 1.0f}
        : ImVec4{0.5f, 0.5f, 0.5f, 0.7f}, "%s", apLabel);

    auto apButton = [&](const char* label, M m) {
        bool active = (apMode == m);
        if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.5f, 0.1f, 1.0f));
        if (ImGui::Button(label)) autopilot.mode = active ? M::Off : m;
        if (active) ImGui::PopStyleColor();
        ImGui::SameLine();
    };
    apButton("Kill Rot [⇧K]", M::Killrot);
    if (nav.navMode == NavMode::Orbit) {
        apButton("Prograde [⇧P]", M::Prograde);
        apButton("Retro [⇧R]",    M::Retrograde);
        apButton("Nrm+ [⇧N]",     M::NormalPlus);
        apButton("Nrm- [⇧M]",     M::NormalMinus);
    }
    if (nav.navMode == NavMode::Mcc) {
        // Suppress MCC DIR active highlight while burn AP is running — it forces
        // MccPlus internally, so lighting both buttons simultaneously is confusing.
        if (nav.mccBurnPhase == MccBurnPhase::Idle) {
            apButton("MCC DIR [⇧G]", M::MccPlus);
        } else {
            if (ImGui::Button("MCC DIR [⇧G]"))
                autopilot.mode = (apMode == M::MccPlus) ? M::Off : M::MccPlus;
            ImGui::SameLine();
        }

        // MCC-C (coarse, main engine) and MCC-F (fine, aux) burn AP buttons.
        // Greyed out during Lambert flip (MCC suppressed); active (green) when running.
        auto mccBurnButton = [&](const char* label, bool isCoarse) {
            const bool active  = nav.mccBurnPhase != MccBurnPhase::Idle
                              && nav.mccBurnCoarse == isCoarse;
            const bool blocked = nav.lambertFlip;
            int pushCount = 0;
            if (active) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.5f, 0.1f, 1.0f));
                ++pushCount;
            } else if (blocked) {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.25f, 0.25f, 0.25f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.25f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(0.4f,  0.4f,  0.4f,  1.0f));
                pushCount = 3;
            }
            if (ImGui::Button(label) && !blocked) {
                if (isCoarse) nav.mccCoarseToggle = true;
                else          nav.mccFineToggle   = true;
            }
            if (pushCount) ImGui::PopStyleColor(pushCount);
            ImGui::SameLine();
        };
        mccBurnButton("MCC-C [⇧C]", true);
        mccBurnButton("MCC-F [⇧F]", false);
    }
    if (nav.navMode == NavMode::Docking && nav.dockTgtIdx >= 0) {
        apButton("NULL V [⇧V]", M::NullV);

        // +V / -V attitude buttons — work standalone or alongside NullV.
        // When NullV is active they set secondaryMode; otherwise they set mode.
        auto rvButton = [&](const char* label, M m) {
            const bool isNullV  = (apMode == M::NullV);
            const bool active   = isNullV ? (autopilot.secondaryMode == m)
                                          : (apMode == m);
            if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.5f, 0.1f, 1.0f));
            if (ImGui::Button(label)) {
                if (isNullV) {
                    autopilot.secondaryMode = active ? M::Off : m;
                } else {
                    // Clear secondary when switching standalone mode.
                    autopilot.secondaryMode = M::Off;
                    autopilot.mode = active ? M::Off : m;
                }
            }
            if (active) ImGui::PopStyleColor();
            ImGui::SameLine();
        };
        rvButton("+V [⇧+]", M::RelVelPlus);
        rvButton("-V [⇧-]", M::RelVelMinus);
    }

    ImGui::PopStyleColor();
    ImGui::End();
}
