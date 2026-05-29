#pragma once

#include "NavState.h"
#include "apeiron/spacecraft/Spacecraft.h"
#include "apeiron/spacecraft/Autopilot.h"
#include "apeiron/render/GltfModel.h"

#include <glm/glm.hpp>
#include <imgui.h>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// NavConsole
//
// Owns the per-frame smoothing state (EMA angular rates/accelerations) and
// renders the centre-bottom Nav Console ImGui window.
//
// Usage each frame (inside the Nav view block):
//   console.update(ship, shipForce, shipMass, frameDt);
//   console.render(posX, posY, width, nav, autopilot, spacecraft, playerIdx, names, mainEngineOn);
// ---------------------------------------------------------------------------
class NavConsole {
public:
    // Accumulate EMA and compute derived quantities.
    // Must be called before render() each frame.
    void update(const Spacecraft&  ship,
                const glm::dvec3&  shipForce,   // body-frame thrust (N)
                double             shipMass,
                float              frameDt);

    using DockPort = apeiron::render::GltfModel::DockingPort;

    // Draw the console ImGui window.
    //   posX, posY   — bottom-left anchor in screen space
    //   width        — window width (== MFD width)
    //   scPorts      — docking port lists, index parallel to spacecraft[]
    void render(float                                           posX,
                float                                           posY,
                float                                           width,
                NavState&                                       nav,
                spacecraft::Autopilot&                          autopilot,
                std::vector<std::unique_ptr<Spacecraft>>&        spacecraft,
                size_t                                          playerIdx,
                const std::vector<std::string>&                 scNames,
                const std::vector<std::vector<DockPort>>&       scPorts,
                bool                                            mainEngineOn,
                const glm::dvec3&                               tcmDv = {});

private:
    // Finite-difference / EMA state
    glm::dvec3 m_prevAngVelInertial{0.0};
    float      m_prevDt            = 0.016f;
    glm::dvec3 m_emaAngVel_degs    {0.0};
    glm::dvec3 m_emaAngAcc_degs    {0.0};

    // Values computed by update(), consumed by render()
    glm::dvec3 m_linAcc_g          {0.0};
    double     m_progradeErrDeg    = 0.0;
    double     m_vsKms             = 0.0;
    double     m_shipMass          = 1.0;  // kg — stored for NullV time/distance estimates
};
