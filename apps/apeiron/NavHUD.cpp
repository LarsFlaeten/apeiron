#include "NavHUD.h"

#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <optional>

void NavHUD::render(const Spacecraft&                              ship,
                    std::vector<std::unique_ptr<Spacecraft>>&      spacecraft,
                    const NavState&                                nav,
                    const glm::dvec3&                              earthWorld,
                    const apeiron::universe::Scene&                scene,
                    const apeiron::render::Camera&                 camera,
                    float                                          W,
                    float                                          H)
{
    ImDrawList* dl = ImGui::GetForegroundDrawList();

    const float  cx         = W * 0.5f;
    const float  cy         = H * 0.5f;
    const ImU32  kHudGreen  = IM_COL32(  0, 210,  75, 180);
    const ImU32  kHudYellow = IM_COL32(220, 200,   0, 210);
    const ImU32  kCyan      = IM_COL32(  0, 200, 255, 200);

    // ---- Center reticle ----
    constexpr float kInner = 5.0f, kOuter = 14.0f;
    dl->AddLine({cx - kOuter, cy}, {cx - kInner, cy}, kHudGreen, 1.5f);
    dl->AddLine({cx + kInner, cy}, {cx + kOuter, cy}, kHudGreen, 1.5f);
    dl->AddLine({cx, cy - kOuter}, {cx, cy - kInner}, kHudGreen, 1.5f);
    dl->AddLine({cx, cy + kInner}, {cx, cy + kOuter}, kHudGreen, 1.5f);

    // ---- Projection helper ----
    glm::vec3  shipRp = scene.origin().toRenderSpace(earthWorld + ship.position());
    glm::mat4  vp     = camera.viewProjection();

    auto projectDir = [&](glm::vec3 dir) -> std::optional<ImVec2> {
        glm::vec4 clip = vp * glm::vec4(shipRp + dir * 1000.0f, 1.0f);
        if (clip.w <= 0.0f) return std::nullopt;
        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        float sx = (ndc.x * 0.5f + 0.5f) * W;
        float sy = (ndc.y * 0.5f + 0.5f) * H;
        if (sx < -50.0f || sx > W + 50.0f || sy < -50.0f || sy > H + 50.0f)
            return std::nullopt;
        return ImVec2{sx, sy};
    };

    // ---- Shared marker drawers ----
    auto drawPrograde = [&](ImVec2 p, ImU32 col) {
        constexpr float r = 14.0f, tick = 8.0f;
        dl->AddCircle(p, r, col, 0, 1.5f);
        dl->AddLine({p.x - r - tick, p.y}, {p.x - r,        p.y}, col, 1.5f);
        dl->AddLine({p.x + r,        p.y}, {p.x + r + tick, p.y}, col, 1.5f);
        dl->AddLine({p.x, p.y - r - tick}, {p.x, p.y - r       }, col, 1.5f);
        dl->AddLine({p.x, p.y + r       }, {p.x, p.y + r + tick}, col, 1.5f);
    };
    auto drawRetrograde = [&](ImVec2 p, ImU32 col) {
        constexpr float r = 14.0f, d = 8.0f;
        dl->AddCircle(p, r, col, 0, 1.5f);
        dl->AddLine({p.x - d, p.y - d}, {p.x + d, p.y + d}, col, 1.5f);
        dl->AddLine({p.x + d, p.y - d}, {p.x - d, p.y + d}, col, 1.5f);
    };

    // ==================================================================
    if (nav.navMode == NavMode::Orbit) {
        // ---- Prograde / retrograde markers ----
        if (glm::length(ship.velocity()) > 1e-9) {
            glm::vec3 progradeDir = glm::normalize(glm::vec3(ship.velocity()));
            if (auto p = projectDir( progradeDir)) drawPrograde (*p, kHudYellow);
            if (auto p = projectDir(-progradeDir)) drawRetrograde(*p, kHudYellow);
        }
    } else {
        // ---- Docking mode ----
        if (nav.dockTgtIdx >= 0) {
            auto& tgt = *spacecraft[static_cast<size_t>(nav.dockTgtIdx)];

            // Relative velocity markers.
            glm::dvec3 relVel = ship.velocity() - tgt.velocity();
            const double rvMag = glm::length(relVel);
            if (rvMag > 1e-9) {
                glm::vec3 rvDir = glm::normalize(glm::vec3(relVel));

                char rvBuf[32];
                double rvMs = rvMag * 1000.0;
                if (rvMs >= 1.0)
                    std::snprintf(rvBuf, sizeof(rvBuf), "%.2f m/s", rvMs);
                else
                    std::snprintf(rvBuf, sizeof(rvBuf), "%.1f cm/s", rvMs * 100.0);

                constexpr float kR = 14.0f, kD = 8.0f;

                // O+ = direction of relative velocity (closing toward target if pointing at it)
                auto drawRelVelPlus = [&](ImVec2 p, ImU32 col) {
                    dl->AddCircle(p, kR, col, 0, 1.5f);
                    dl->AddLine({p.x - kD, p.y}, {p.x + kD, p.y}, col, 1.5f);
                    dl->AddLine({p.x, p.y - kD}, {p.x, p.y + kD}, col, 1.5f);
                    dl->AddText({p.x + kR + 4.0f, p.y - 6.0f}, col, rvBuf);
                };
                // OX = opposite direction
                auto drawRelVelMinus = [&](ImVec2 p, ImU32 col) {
                    dl->AddCircle(p, kR, col, 0, 1.5f);
                    dl->AddLine({p.x - kD, p.y - kD}, {p.x + kD, p.y + kD}, col, 1.5f);
                    dl->AddLine({p.x + kD, p.y - kD}, {p.x - kD, p.y + kD}, col, 1.5f);
                    dl->AddText({p.x + kR + 4.0f, p.y - 6.0f}, col, rvBuf);
                };

                if (auto p = projectDir( rvDir)) drawRelVelPlus (*p, kCyan);
                if (auto p = projectDir(-rvDir)) drawRelVelMinus(*p, kCyan);
            }

            // Target box with distance label.
            glm::dvec3 tgtWorld = earthWorld + tgt.position();
            glm::vec3  tgtRp    = scene.origin().toRenderSpace(tgtWorld);
            glm::vec4  clip     = vp * glm::vec4(tgtRp, 1.0f);
            if (clip.w > 0.0f) {
                glm::vec3 ndc = glm::vec3(clip) / clip.w;
                float sx = (ndc.x * 0.5f + 0.5f) * W;
                float sy = (ndc.y * 0.5f + 0.5f) * H;
                if (sx > -200.0f && sx < W + 200.0f &&
                    sy > -200.0f && sy < H + 200.0f) {
                    constexpr float kHalf = 40.0f;
                    dl->AddRect({sx - kHalf, sy - kHalf},
                                {sx + kHalf, sy + kHalf},
                                kCyan, 0.0f, 0, 1.5f);
                    double distM = glm::length(ship.position() - tgt.position()) * 1000.0;
                    char buf[32];
                    if (distM < 1000.0)
                        std::snprintf(buf, sizeof(buf), "%.1f m", distM);
                    else
                        std::snprintf(buf, sizeof(buf), "%.3f km", distM * 1e-3);
                    dl->AddText({sx - kHalf, sy + kHalf + 4.0f}, kCyan, buf);
                }
            }
        }
    }
}
