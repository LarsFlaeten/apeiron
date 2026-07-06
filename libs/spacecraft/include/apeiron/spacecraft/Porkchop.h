#pragma once

#include <astro/State.h>

#include <glm/glm.hpp>
#include <vector>
#include <functional>

namespace spacecraft {

// ---------------------------------------------------------------------------
// Parameters for a single-leg porkchop plot.
//
// All times in seconds past J2000 (SPICE ET).
// ---------------------------------------------------------------------------
struct PorkchopParams {
    int    departureBody = 399;   // NAIF ID  (399 = Earth)
    int    arrivalBody   = 4;     // NAIF ID  (4   = Mars barycenter)
    int    centralBody   = 10;    // NAIF ID  (10  = Sun)
    double muCentral     = 0.0;   // km³/s² — queried from SPICE if 0

    double t0     = 0.0;    // departure window start (ET)
    double t1     = 0.0;    // departure window end   (ET)
    double tofMin = 0.0;    // minimum TOF (s)
    double tofMax = 0.0;    // maximum TOF (s)

    int nDep = 60;    // grid columns (departure dates)
    int nTof = 60;    // grid rows    (TOF values)

    // Optional departure-state override.
    // When true, departureR/V are used directly instead of querying SPICE for
    // the departure body.  Required for cislunar TLI where the departure point
    // is the ship in its parking orbit, not a planet centre (which would return
    // the degenerate (0,0,0) when departureBody == centralBody).
    bool       useDepartureOverride = false;
    glm::dvec3 departureR { 0.0 };   // km,   relative to centralBody
    glm::dvec3 departureV { 0.0 };   // km/s, relative to centralBody

    // Optional arrival-state override function.
    // When set, called with the arrival ET (seconds past J2000) and returns
    // the arrival body state relative to centralBody.  Takes priority over the
    // SPICE arrivalBody lookup.  Use for targets that are not SPICE bodies
    // (e.g. Gateway on its NRHO, computed via vehicleStateAtEt).
    std::function<astro::PosState(double etArr)> arrivalOverrideFn;
};

// ---------------------------------------------------------------------------
// Result of computePorkchop().
//
// Grid is stored row-major: index = iTof * nDep + iDep.
// dv1     — departure ΔV (km/s), i.e. |v_lambert_dep − v_body_dep|
// dv2     — arrival  ΔV (km/s), i.e. |v_lambert_arr − v_body_arr|
// dvTotal — dv1 + dv2
// Cells where Lambert did not converge have dvTotal = kNoSolution.
// ---------------------------------------------------------------------------
struct PorkchopData {
    static constexpr float kNoSolution = 1e9f;

    PorkchopParams params;

    std::vector<float> dv1;
    std::vector<float> dv2;
    std::vector<float> dvTotal;

    float dvMin = kNoSolution;
    float dvMax = 0.0f;

    // Convenience accessor.
    float get(const std::vector<float>& grid, int iDep, int iTof) const {
        return grid[iTof * params.nDep + iDep];
    }

    // ET of the departure axis tick iDep.
    double depET(int iDep) const;

    // TOF (seconds) of the TOF axis tick iTof.
    double tofS(int iTof) const;
};

// ---------------------------------------------------------------------------
// Compute a porkchop grid.  Requires SPICE kernels to already be loaded
// (LSK + PCK + SPK covering departureBody, arrivalBody, centralBody).
//
// progress — optional callback, called with fraction in [0, 1].
// ---------------------------------------------------------------------------
PorkchopData computePorkchop(const PorkchopParams& p,
                              std::function<void(float)> progress = nullptr);

} // namespace spacecraft
