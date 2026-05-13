#pragma once

#include "OBCEventQueue.h"

namespace spacecraft { class Autopilot; }

#include <astro/State.h>
#include <astro/Time.h>

#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// MFDContext — per-frame snapshot of simulator state for MFD consumption.
//
// Built once per frame in main.cpp from SPICE queries and spacecraft state,
// then passed to every MFD's update() method.  MFDs read from it; they do
// not write back (except via eventQueue, which is logically separate).
//
// Replaces the per-MFD push pattern where main.cpp called
//   orbitalMFD.update(state, et, mu, radius)
//   mapMFD.update(pos, vel, cfg, rot, sunDir)
//   transferMFD.updateShipState(...) / updateBurnParams(...) / ...
// with a single
//   mfd.update(ctx)
// call for each MFD.
// ---------------------------------------------------------------------------
// One entry per solar-system body available for transfer planning.
// Populated by main.cpp from the loaded scenario; TransferMFD reads it.
struct BodyEntry {
    std::string naifName;   // as loaded, e.g. "JUPITER BARYCENTER"
    std::string shortName;  // first word, e.g. "JUPITER"
    int         naifId = 0; // SPICE integer NAIF code
};

struct MFDContext {
    // ---- Time ---------------------------------------------------------------
    astro::EphemerisTime currentEt;    // canonical simulation ET
    double               simElapsed = 0.0; // seconds since scenario epoch

    // ---- Reference body -----------------------------------------------------
    std::string refBodyName;           // e.g. "EARTH", "MARS"
    double      refBodyMu       = 0.0; // km³/s²
    double      refBodyRadiusKm = 0.0; // equatorial radius (km)

    // ---- Ship state: relative to reference body (ECLIPJ2000, km / km/s) ----
    astro::PosState shipRelRef;

    // ---- Ship state: geocentric ECI (ECLIPJ2000) — for TransferMFD depart. -
    glm::dvec3 shipGeoR { 0.0 };
    glm::dvec3 shipGeoV { 0.0 };

    // ---- Ship state: heliocentric (Sun-centred ECLIPJ2000) — TransferMFD ---
    glm::dvec3 shipHelioR { 0.0 };
    glm::dvec3 shipHelioV { 0.0 };

    // ---- Body-fixed map data ------------------------------------------------
    // Rotation matrix ECLIPJ2000 → IAU_<refBody> at currentEt.
    glm::dmat3 inertialToBody  { 1.0 };
    // Body rotation rate (rad/s, positive = prograde spin).
    double     mapRotRateRadSec = 0.0;
    // Unit vector toward the Sun in body-centred ECLIPJ2000.
    glm::dvec3 sunDirInertial  { 1.0, 0.0, 0.0 };

    // ---- Propulsion ---------------------------------------------------------
    double mainThrustN = 0.0; // main engine thrust (N)
    double shipMassKg  = 1.0; // total spacecraft mass (kg)

    // ---- Event queue (non-owning) -------------------------------------------
    OBCEventQueue* eventQueue = nullptr;

    // ---- Autopilot (non-owning) — for burn arm/disarm from MFDs -------------
    spacecraft::Autopilot* autopilot = nullptr;

    // ---- Bodies available for transfer planning (populated from scenario) ---
    // Excludes Sun and moons; contains planets and barycenters only.
    std::vector<BodyEntry> solarSystemBodies;
};
