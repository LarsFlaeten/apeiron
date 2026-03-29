#pragma once

#include "MFD.h"

#include "astro/OrbitElements.h"
#include "astro/State.h"
#include "astro/Time.h"

// ---------------------------------------------------------------------------
// OrbitalMFD — displays classical Keplerian orbital elements.
//
// Call update() once per frame (or whenever state changes) with the current
// spacecraft position/velocity state and the primary body's GM.
// render() is called by MFDPanel and draws labelled rows via ImDrawList.
// ---------------------------------------------------------------------------
class OrbitalMFD : public MFDApp {
public:
    const char* name() const override { return "ORB"; }

    // Recompute orbital elements from the current state vector.
    // mu          — gravitational parameter of the primary body (km³/s²)
    // bodyRadiusKm — equatorial radius used to convert apoapsis/periapsis to altitude
    void update(const astro::PosState&    state,
                const astro::EphemerisTime& et,
                double mu,
                double bodyRadiusKm);

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

private:
    double m_altKm    = 0.0;
    double m_velKms   = 0.0;
    double m_apoAltKm = 0.0;
    double m_perAltKm = 0.0;
    double m_incDeg   = 0.0;
    double m_raanDeg  = 0.0;
    double m_eccen    = 0.0;
    double m_argpeDeg = 0.0;
    double m_trueDeg   = 0.0;
    bool   m_trueValid = false;
    double m_periodMin = 0.0;
};
