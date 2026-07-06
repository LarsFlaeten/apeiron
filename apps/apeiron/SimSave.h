#pragma once

#include "apeiron/spacecraft/Spacecraft.h"
#include "TransferMFD.h"
#include "CislunarMFD.h"

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
    double      simElapsed     = 0.0;  // seconds since scenario epoch
    double      simSpeedTarget = 1.0;
    std::string refBodyName;           // NAIF name of the selected reference body (e.g. "MARS")

    struct ScState {
        glm::dvec3 position;       // km   (ECI geocentric ECLIPJ2000)
        glm::dvec3 velocity;       // km/s
        glm::dquat attitude;       // body→inertial quaternion (w,x,y,z)
        glm::dvec3 angularVelocity;// rad/s, inertial frame
    };
    std::vector<ScState> spacecraft;  // same order as main.cpp spacecraft[]

    TransferPlanSnapshot  plan;         // plan.valid=false if no plan was active
    CislunarPlanSnapshot  cislunarPlan; // cislunarPlan.valid=false if no plan was active

    // Docking state — only present when active spacecraft was in HardCapture at save time.
    struct DockingState {
        bool active         = false; // true = a docked pair was saved
        int  activeScIdx    = 0;     // index into spacecraft[] (probe / player)
        int  passiveScIdx   = 1;     // index into spacecraft[] (drogue / station)
        int  activePortIdx  = 0;     // port index on active  spacecraft
        int  passivePortIdx = 0;     // port index on passive spacecraft
    } docking;

    // Scheduled OBC events — empty if none were active at save time.
    struct SavedEvent {
        std::string name;
        double      eventET      = 0.0;
        bool        ann10min     = false;
        bool        ann5min      = false;
        bool        ann1min      = false;
        bool        ann10s       = false;
        bool        ann0         = false;
        bool        countdown10s = false;
    };
    std::vector<SavedEvent> events;
};

// Returns true on success.  Creates parent directories as needed.
bool saveSimState(const std::string& path, const SimSaveData& data);

// Returns true on success.  On failure data is unchanged.
bool loadSimState(const std::string& path, SimSaveData& data);
