#pragma once

#include "NavState.h"
#include "Spacecraft.h"
#include "apeiron/render/GltfModel.h"

#include <apeiron/render/Camera.h>
#include <apeiron/universe/Scene.h>

#include <glm/glm.hpp>
#include <imgui.h>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// NavHUD
//
// Stateless renderer for the helmet HUD overlay (drawn on the foreground
// draw list, on top of everything).
//
// Orbit mode:  prograde / retrograde markers.
// Docking mode: relative-velocity markers (O+ / OX) + target box with
//               distance label.
// ---------------------------------------------------------------------------
class NavHUD {
public:
    // Draw everything for this frame.
    //   ship        — player spacecraft
    //   spacecraft  — full array (for docking target lookup)
    //   nav         — current nav/docking state
    //   earthWorld  — Earth position in ECLIPJ2000 render space (km)
    //   scene       — for toRenderSpace()
    //   camera      — for viewProjection()
    //   screenW/H   — ImGui display size
    using DockPort = apeiron::render::GltfModel::DockingPort;

    void render(const Spacecraft&                              ship,
                std::vector<std::unique_ptr<Spacecraft>>&      spacecraft,
                const NavState&                                nav,
                const glm::dvec3&                              earthWorld,
                const apeiron::universe::Scene&                scene,
                const apeiron::render::Camera&                 camera,
                float                                          screenW,
                float                                          screenH,
                const std::vector<std::string>&                scNames,
                const std::vector<std::vector<DockPort>>&      scPorts,
                const char*                                    refBodyName    = "EARTH",
                const glm::dvec3&                              tcmDv          = {},
                const glm::dvec3&                              shipRelRefV    = {},
                const glm::dvec3&                              shipRelRefR    = {});
};
