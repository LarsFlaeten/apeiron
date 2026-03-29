#pragma once

#include "MFD.h"

#include "astro/OrbitElements.h"
#include "astro/State.h"
#include "astro/Time.h"

// ---------------------------------------------------------------------------
// OrbitalMFD — Keplerian orbital elements + orbit diagram.
//
// setContext() — call once (or when the reference changes) to name the
//               reference body and coordinate frame shown in the header.
// update()     — call every frame with the current state vector.
// render()     — called by MFDPanel; draws via ImDrawList.
//
// Layout inside the content area (top → bottom):
//   header row  — reference body + frame name
//   4 data rows — two columns each (Alt/Vel, Apo/Per, Inc/Ecc, h/T)
//   orbit diagram — fills remaining height, centred horizontally
// ---------------------------------------------------------------------------
class OrbitalMFD : public MFDApp {
public:
    const char* name() const override { return "ORB"; }

    // Set the reference body name and coordinate frame label.
    void setContext(const char* refName, const char* frameName);

    // Recompute orbital elements from the current state vector.
    //   mu           — GM of the reference body (km³/s²)
    //   bodyRadiusKm — equatorial radius for altitude conversion
    void update(const astro::PosState&     state,
                const astro::EphemerisTime& et,
                double mu,
                double bodyRadiusKm);

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

private:
    // Draws the 2-D orbit diagram in a square of side diagSize at diagOrigin.
    void renderDiagram(ImDrawList* dl, ImVec2 diagOrigin, float diagSize) const;

    // Context (static, set once)
    const char* m_refName   = "EARTH";
    const char* m_frameName = "ECLIPJ2000";

    // Derived quantities updated every frame
    double m_altKm      = 0.0;
    double m_velKms     = 0.0;
    double m_apoAltKm   = 0.0;
    double m_perAltKm   = 0.0;
    double m_incDeg     = 0.0;
    double m_raanDeg    = 0.0;
    double m_eccen      = 0.0;
    double m_argpeDeg   = 0.0;
    double m_trueDeg    = 0.0;
    bool   m_trueValid  = false;
    double m_periodMin  = 0.0;
    double m_angMomKm2s = 0.0;  // specific angular momentum  (km²/s)
    double m_smaKm      = 0.0;  // semi-major axis             (km)
    double m_bodyRadiusKm = 0.0;
};
