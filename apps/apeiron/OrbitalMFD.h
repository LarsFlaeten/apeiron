#pragma once

#include "MFD.h"

#include "astro/State.h"
#include "astro/Time.h"
#include "astro/OrbitElements.h"   // only for meanAnomalyFromTrueAnomaly

#include <glm/glm.hpp>
#include <string>
#include <vector>

struct MFDFrame {
    const char* name;   // display name, e.g. "ECLIPJ2000"
    glm::dmat3  rot;    // rotation: ECLIPJ2000 vectors → this frame
};

// ---------------------------------------------------------------------------
// OrbitalMFD
//
// Computes orbital elements directly from the state vector (no Kepler solver),
// so it handles all eccentricities: circular, elliptic, near-parabolic (e→1),
// and hyperbolic (e > 1) without crashing.
//
// Layout inside the content area (top → bottom):
//   orbit diagram  — right side, full content height, drawn first
//   header row     — REF • frame (drawn on top)
//   up to 7 rows   — two columns, drawn on top of diagram
// ---------------------------------------------------------------------------
class OrbitalMFD : public MFDApp {
public:
    const char* name() const override { return "ORB"; }

    void setContext(const char* refName, const char* frameName);

    // Provide the list of frames the FRM button cycles through.
    // The first entry is shown on start-up; identity rot = ECLIPJ2000.
    void setFrames(std::vector<MFDFrame> frames);

    // Called by main.cpp each frame; returns the new body name the user typed,
    // or an empty string when there is no pending request.
    std::string consumePendingRef();

    // Recompute every frame.
    //   mu           — GM of the reference body (km³/s²)
    //   bodyRadiusKm — equatorial radius for altitude conversion
    void update(const astro::PosState&     state,
                const astro::EphemerisTime& et,
                double mu,
                double bodyRadiusKm);

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel(int slot) const override;
    void        onLeft   (int slot) override;

private:
    void renderDiagram(ImDrawList* dl, ImVec2 diagOrigin, float diagSize) const;

    const char* m_refName   = "EARTH";
    const char* m_frameName = "ECLIPJ2000";  // fallback when m_frames is empty

    // REF input overlay (slot 0)
    bool        m_refInputActive = false;
    char        m_refInputBuf[32]{};
    std::string m_pendingRef;

    // FRM cycling (slot 1)
    std::vector<MFDFrame> m_frames;
    int                   m_frameIdx = 0;

    // --- updated every frame ---
    double m_altKm        = 0.0;
    double m_velKms       = 0.0;
    double m_apoAltKm     = 0.0;   // < 0 signals "no apoapsis" (hyperbolic)
    double m_perAltKm     = 0.0;
    double m_incDeg       = 0.0;
    double m_raanDeg      = 0.0;
    double m_eccen        = 0.0;
    double m_argpeDeg     = 0.0;
    double m_trueDeg      = 0.0;
    bool   m_trueValid    = false;
    double m_periodMin    = -1.0;  // negative = no period
    double m_angMomKm2s   = 0.0;
    double m_smaKm        = 0.0;   // negative for hyperbolic
    double m_tToApoSec    = -1.0;
    double m_tToPerSec    = -1.0;
    double m_hypVinfKms   = 0.0;   // v∞ for hyperbolic orbits
    bool   m_isHyperbolic = false;
    double m_bodyRadiusKm = 0.0;

    // Scale is frozen when e > kFreezeEcc so the diagram doesn't zoom out to nothing.
    // Stores the SMA (km) used for the last "normal" scale computation.
    double m_frozenSmaKm  = 0.0;
    static constexpr double kFreezeEcc = 0.90;
};
