#include "NavHUD.h"

#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <cmath>
#include <optional>
#include <string>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
namespace {

// Project a world point to screen coords.
// Returns the raw (possibly off-screen) sx/sy and the w value.
// Caller checks w > 0 for "in front of camera".
struct Projected {
    float sx, sy;
    float w;
    bool onScreen(float W, float H, float margin = 50.0f) const {
        return w > 0.0f
            && sx >= -margin && sx <= W + margin
            && sy >= -margin && sy <= H + margin;
    }
};

Projected project(const glm::mat4& vp, const glm::vec3& worldPos, float W, float H)
{
    glm::vec4 clip = vp * glm::vec4(worldPos, 1.0f);
    if (clip.w <= 0.0f) return {0.0f, 0.0f, clip.w};
    glm::vec3 ndc = glm::vec3(clip) / clip.w;
    float sx = (ndc.x * 0.5f + 0.5f) * W;
    float sy = (ndc.y * 0.5f + 0.5f) * H;
    return {sx, sy, clip.w};
}

// Clamp a point that is (possibly) off-screen to the screen edge.
// Returns the edge position and the normalised direction FROM edge TOWARD the off-screen point.
struct EdgeHit {
    ImVec2     pos;   // clamped position on screen boundary
    ImVec2     dir;   // unit vector pointing toward the off-screen target
};

EdgeHit clampToEdge(float sx, float sy, float W, float H, float margin)
{
    float cx = W * 0.5f;
    float cy = H * 0.5f;
    float dx = sx - cx;
    float dy = sy - cy;

    // Find the smallest t > 0 such that (cx + t*dx, cy + t*dy) hits a boundary.
    float t = 1e9f;
    float L = margin, R = W - margin, T = margin, B = H - margin;
    if (dx >  1e-4f) t = std::min(t, (R - cx) / dx);
    if (dx < -1e-4f) t = std::min(t, (L - cx) / dx);
    if (dy >  1e-4f) t = std::min(t, (B - cy) / dy);
    if (dy < -1e-4f) t = std::min(t, (T - cy) / dy);

    float ex = cx + t * dx;
    float ey = cy + t * dy;
    float len = std::sqrt(dx * dx + dy * dy);
    return { ImVec2{ex, ey}, ImVec2{dx / len, dy / len} };
}

// Draw a filled arrow triangle at pos, pointing in dir.
void drawArrow(ImDrawList* dl, ImVec2 pos, ImVec2 dir, float size, ImU32 col)
{
    // Perpendicular to dir.
    ImVec2 perp{-dir.y, dir.x};
    ImVec2 tip  { pos.x + dir.x  * size,        pos.y + dir.y  * size        };
    ImVec2 base1{ pos.x - perp.x * size * 0.5f, pos.y - perp.y * size * 0.5f };
    ImVec2 base2{ pos.x + perp.x * size * 0.5f, pos.y + perp.y * size * 0.5f };
    dl->AddTriangleFilled(tip, base1, base2, col);
}

// Format a relative speed value.
void fmtSpeed(char* buf, size_t sz, double rvMs)
{
    if (rvMs >= 1.0)
        std::snprintf(buf, sz, "%.2f m/s", rvMs);
    else
        std::snprintf(buf, sz, "%.1f cm/s", rvMs * 100.0);
}

} // namespace

// ---------------------------------------------------------------------------
void NavHUD::render(const Spacecraft&                              ship,
                    std::vector<std::unique_ptr<Spacecraft>>&      spacecraft,
                    const NavState&                                nav,
                    const glm::dvec3&                              earthWorld,
                    const apeiron::universe::Scene&                scene,
                    const apeiron::render::Camera&                 camera,
                    float                                          W,
                    float                                          H,
                    const std::vector<std::string>&                scNames,
                    const std::vector<std::vector<DockPort>>&      scPorts,
                    const char*                                    refBodyName,
                    const glm::dvec3&                              mccDv)
{
    ImDrawList* dl  = ImGui::GetForegroundDrawList();
    const float cx  = W * 0.5f;
    const float cy  = H * 0.5f;

    const ImU32 kHudGreen  = IM_COL32(  0, 210,  75, 180);
    const ImU32 kHudYellow = IM_COL32(220, 200,   0, 210);
    const ImU32 kCyan      = IM_COL32(  0, 200, 255, 200);
    const ImU32 kViolet    = IM_COL32(180, 100, 255, 200);

    constexpr float kEdgeMargin = 28.0f;  // px inset from window edge for clamping

    // ---- Center reticle ----
    constexpr float kInner = 5.0f, kOuter = 14.0f;
    dl->AddLine({cx - kOuter, cy}, {cx - kInner, cy}, kHudGreen, 1.5f);
    dl->AddLine({cx + kInner, cy}, {cx + kOuter, cy}, kHudGreen, 1.5f);
    dl->AddLine({cx, cy - kOuter}, {cx, cy - kInner}, kHudGreen, 1.5f);
    dl->AddLine({cx, cy + kInner}, {cx, cy + kOuter}, kHudGreen, 1.5f);

    // ---- Projection setup ----
    glm::vec3 shipRp = scene.origin().toRenderSpace(earthWorld + ship.position());
    glm::mat4 vp     = camera.viewProjection();

    // Project a direction 1000 km out from the ship.
    auto projectDir = [&](glm::vec3 dir) -> Projected {
        return project(vp, shipRp + dir * 1000.0f, W, H);
    };

    // ---- Standard marker drawers (on-screen) ----
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
    // Orbit normal markers: circle + center dot (N+), circle + inner ring (N-).
    auto drawNormalPlus = [&](ImVec2 p, ImU32 col) {
        constexpr float r = 14.0f;
        dl->AddCircle(p, r, col, 0, 1.5f);
        dl->AddCircleFilled(p, 3.5f, col);
        dl->AddText({p.x + r + 4.0f, p.y - 6.0f}, col, "N+");
    };
    auto drawNormalMinus = [&](ImVec2 p, ImU32 col) {
        constexpr float r = 14.0f;
        dl->AddCircle(p, r, col, 0, 1.5f);
        dl->AddCircle(p, 5.0f, col, 0, 1.5f);
        dl->AddText({p.x + r + 4.0f, p.y - 6.0f}, col, "N-");
    };

    // ==================================================================
    if (nav.navMode == NavMode::Orbit) {
        // ---- Orbit: prograde / retrograde / normal+/normal- ----
        const glm::dvec3 vel  = ship.velocity();
        const glm::dvec3 pos  = ship.position();
        if (glm::length(vel) > 1e-9) {
            glm::vec3 pgDir = glm::normalize(glm::vec3(vel));
            auto pg  = projectDir( pgDir);
            auto ret = projectDir(-pgDir);
            if (pg.onScreen(W, H))  drawPrograde ({pg.sx,  pg.sy},  kHudYellow);
            if (ret.onScreen(W, H)) drawRetrograde({ret.sx, ret.sy}, kHudYellow);

            // Orbit normal markers (always shown in orbit mode when normal is defined).
            glm::dvec3 h = glm::cross(pos, vel);
            if (glm::length(h) > 1e-12) {
                glm::vec3 normDir = glm::normalize(glm::vec3(h));
                auto np  = projectDir( normDir);
                auto nm  = projectDir(-normDir);
                if (np.onScreen(W, H))  drawNormalPlus ({np.sx,  np.sy},  kViolet);
                if (nm.onScreen(W, H))  drawNormalMinus({nm.sx,  nm.sy},  kViolet);
            }
        }
    } else {
        // ==============================================================
        // Docking mode
        // ==============================================================
        if (nav.dockTgtIdx < 0) return;

        auto& tgt = *spacecraft[static_cast<size_t>(nav.dockTgtIdx)];
        const char* tgtName = (nav.dockTgtIdx < static_cast<int>(scNames.size()))
                              ? scNames[nav.dockTgtIdx].c_str() : "TGT";

        // ---- Relative velocity markers ----
        glm::dvec3 relVel = ship.velocity() - tgt.velocity();
        const double rvMag = glm::length(relVel);

        if (rvMag > 1e-9) {
            glm::vec3 rvDir = glm::normalize(glm::vec3(relVel));

            char speedBuf[32];
            fmtSpeed(speedBuf, sizeof(speedBuf), rvMag * 1000.0);

            constexpr float kR = 14.0f, kD = 8.0f;

            // Draw O+ (closing) and OX (opening) with target name + speed label.
            // If off-screen: draw edge arrow instead.
            struct MarkerDef {
                glm::vec3   dir;
                const char* prefix;  // "V+" or "V-"
                bool        isPlus;
            } markers[2] = {
                {  rvDir, "V+", true  },
                { -rvDir, "V-", false },
            };

            for (auto& mk : markers) {
                auto proj = projectDir(mk.dir);

                if (proj.w > 0.0f && proj.onScreen(W, H, 50.0f)) {
                    // --- On-screen ---
                    ImVec2 p{proj.sx, proj.sy};
                    dl->AddCircle(p, kR, kCyan, 0, 1.5f);
                    if (mk.isPlus) {
                        // O+ : inner cross
                        dl->AddLine({p.x - kD, p.y}, {p.x + kD, p.y}, kCyan, 1.5f);
                        dl->AddLine({p.x, p.y - kD}, {p.x, p.y + kD}, kCyan, 1.5f);
                    } else {
                        // OX : inner X
                        dl->AddLine({p.x - kD, p.y - kD}, {p.x + kD, p.y + kD}, kCyan, 1.5f);
                        dl->AddLine({p.x + kD, p.y - kD}, {p.x - kD, p.y + kD}, kCyan, 1.5f);
                    }
                    // Speed + target name to the right
                    char lbl[64];
                    std::snprintf(lbl, sizeof(lbl), "%s  %s  %s", mk.prefix, speedBuf, tgtName);
                    dl->AddText({p.x + kR + 4.0f, p.y - 6.0f}, kCyan, lbl);
                } else if (proj.w > 0.0f) {
                    // --- Off-screen: edge indicator ---
                    auto edge = clampToEdge(proj.sx, proj.sy, W, H, kEdgeMargin);
                    drawArrow(dl, edge.pos, edge.dir, 9.0f, kCyan);

                    // Label: "V+ 0.12 m/s  ISS" placed perpendicular to arrow, offset slightly
                    char lbl[64];
                    std::snprintf(lbl, sizeof(lbl), "%s %s  %s", mk.prefix, speedBuf, tgtName);
                    // Offset the text away from the edge so it doesn't clip
                    ImVec2 textPos{
                        edge.pos.x + edge.dir.x * 14.0f - 10.0f,
                        edge.pos.y + edge.dir.y * 14.0f - 6.0f
                    };
                    dl->AddText(textPos, kCyan, lbl);
                }
                // if w <= 0: behind camera, skip entirely
            }
        }

        // ---- Target box with distance label ----
        // When a specific docking port is selected, aim at the port instead of CoM.
        {
            glm::vec3 tgtRp;
            double    distM;
            std::string tgtLabel = tgtName;

            if (nav.dockPortIdx >= 0 &&
                nav.dockTgtIdx < static_cast<int>(scPorts.size())) {
                const auto& ports = scPorts[static_cast<size_t>(nav.dockTgtIdx)];
                if (nav.dockPortIdx < static_cast<int>(ports.size())) {
                    const auto& port = ports[nav.dockPortIdx];
                    // Port world pos = tgt render pos + tgt attitude * port offset (m→km)
                    glm::vec3 tgtCoMRp = scene.origin().toRenderSpace(earthWorld + tgt.position());
                    glm::mat3 tgtRot   = glm::mat3_cast(glm::fquat(tgt.attitude()));
                    tgtRp = tgtCoMRp + tgtRot * (port.posM * 1e-3f);
                    glm::vec3 portWorldKm = glm::vec3(tgt.position()) + glm::vec3(glm::mat3_cast(tgt.attitude()) * glm::dvec3(port.posM * 1e-3f));
                    distM = glm::length(glm::vec3(ship.position()) - portWorldKm) * 1000.0;
                    tgtLabel = std::string(tgtName) + "/" + port.label;
                } else {
                    tgtRp = scene.origin().toRenderSpace(earthWorld + tgt.position());
                    distM = glm::length(ship.position() - tgt.position()) * 1000.0;
                }
            } else {
                tgtRp = scene.origin().toRenderSpace(earthWorld + tgt.position());
                distM = glm::length(ship.position() - tgt.position()) * 1000.0;
            }
            auto proj = project(vp, tgtRp, W, H);
            char distBuf[48];
            if (distM < 1000.0)
                std::snprintf(distBuf, sizeof(distBuf), "%.1f m", distM);
            else
                std::snprintf(distBuf, sizeof(distBuf), "%.3f km", distM * 1e-3);

            if (proj.w > 0.0f) {
                if (proj.onScreen(W, H, 200.0f)) {
                    constexpr float kHalf = 40.0f;
                    dl->AddRect({proj.sx - kHalf, proj.sy - kHalf},
                                {proj.sx + kHalf, proj.sy + kHalf},
                                kCyan, 0.0f, 0, 1.5f);
                    dl->AddText({proj.sx - kHalf, proj.sy + kHalf + 4.0f}, kCyan, distBuf);
                } else {
                    auto edge = clampToEdge(proj.sx, proj.sy, W, H, kEdgeMargin);
                    drawArrow(dl, edge.pos, edge.dir, 9.0f, kCyan);
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "%s  %s", tgtLabel.c_str(), distBuf);
                    ImVec2 textPos{
                        edge.pos.x + edge.dir.x * 14.0f - 10.0f,
                        edge.pos.y + edge.dir.y * 14.0f - 6.0f
                    };
                    dl->AddText(textPos, kCyan, buf);
                }
            }
        }
    }

    // ---- REF body indicator (top-right corner) ----
    if (refBodyName && refBodyName[0]) {
        char refBuf[32];
        std::snprintf(refBuf, sizeof(refBuf), "REF %s", refBodyName);
        const ImVec2 refSz = ImGui::CalcTextSize(refBuf);
        dl->AddText({ W - refSz.x - 8.0f, 8.0f }, kHudGreen, refBuf);
    }

    // ---- MCC direction marker (orange diamond + "MCC" label) ----
    // Drawn whenever TransferMFD has a valid MCC solution (mccDv != 0).
    // Direction is heliocentric ECLIPJ2000 — same inertial frame as the camera.
    const double mccDvMag = glm::length(mccDv);
    if (mccDvMag > 1e-9) {
        const ImU32 kMccOrange = IM_COL32(255, 140, 0, 230);
        glm::vec3 mccDir = glm::normalize(glm::vec3(mccDv));
        auto mcc = projectDir(mccDir);
        if (mcc.onScreen(W, H)) {
            // Diamond shape (rotated square).
            constexpr float r = 12.0f;
            ImVec2 p { mcc.sx, mcc.sy };
            dl->AddLine({p.x,     p.y - r}, {p.x + r, p.y    }, kMccOrange, 1.5f);
            dl->AddLine({p.x + r, p.y    }, {p.x,     p.y + r}, kMccOrange, 1.5f);
            dl->AddLine({p.x,     p.y + r}, {p.x - r, p.y    }, kMccOrange, 1.5f);
            dl->AddLine({p.x - r, p.y    }, {p.x,     p.y - r}, kMccOrange, 1.5f);
            // Label
            char mccBuf[24];
            std::snprintf(mccBuf, sizeof(mccBuf), "MCC %.1f m/s", mccDvMag * 1000.0);
            dl->AddText({p.x + r + 4.0f, p.y - 6.0f}, kMccOrange, mccBuf);
        }
    }
}
