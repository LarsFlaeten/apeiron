#pragma once

#include "Spacecraft.h"
#include "TransferMFD.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// SimSave — quicksave / quickload for simulation state + TransferMFD plan.
//
// File format: TOML
//   data/saves/quicksave.toml  (default path)
//
// Save captures:
//   - Simulation elapsed time (seconds from scenario epoch)
//   - Time-warp speed target
//   - Position / velocity / attitude / angular-velocity for all spacecraft
//   - TransferMFD plan (window params, selected cell, parking-altitude index)
//
// On load the porkchop grid is re-computed from the saved parameters; the
// selected-cell detail is re-resolved.  This means loading takes a moment
// for the Lambert grid computation, same as pressing COMP.
// ---------------------------------------------------------------------------

struct SimSaveData
{
    double simElapsed     = 0.0;  // seconds since scenario epoch
    double simSpeedTarget = 1.0;

    struct ScState {
        glm::dvec3 position;       // km   (ECI geocentric ECLIPJ2000)
        glm::dvec3 velocity;       // km/s
        glm::dquat attitude;       // body→inertial quaternion (w,x,y,z)
        glm::dvec3 angularVelocity;// rad/s, inertial frame
    };
    std::vector<ScState> spacecraft;  // same order as main.cpp spacecraft[]

    TransferPlanSnapshot plan;        // plan.valid=false if no plan was active
};

// Returns true on success.  Creates parent directories as needed.
bool saveSimState(const std::string& path, const SimSaveData& data);

// Returns true on success.  On failure data is unchanged.
bool loadSimState(const std::string& path, SimSaveData& data);

// Save / load just the transfer plan (independent of sim state).
bool savePlanSnapshot(const std::string& path, const TransferPlanSnapshot& snap);
bool loadPlanSnapshot(const std::string& path, TransferPlanSnapshot& snap);
