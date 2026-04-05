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

    // Set the list of target names for the TGT button to cycle through.
    // Index -1 = no target. Call once after spacecraft are created.
    void setTargets(std::vector<std::string> names);

    // Returns the currently selected target index (-1 = none).
    int targetIndex() const { return m_tgtIdx; }

    // Recompute every frame.
    //   mu           — GM of the reference body (km³/s²)
    //   bodyRadiusKm — equatorial radius for altitude conversion
    void update(const astro::PosState&     state,
                const astro::EphemerisTime& et,
                double mu,
                double bodyRadiusKm);

    // Optional: provide target state this frame. Pass empty PosState to clear.
    void updateTarget(const astro::PosState& state, double mu, double bodyRadiusKm);
    void clearTarget();

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel(int slot) const override;
    void        onLeft   (int slot) override;

private:
    // Non-const: updates m_diagViewRot from mouse input.
    void renderDiagram(ImDrawList* dl, ImVec2 diagOrigin, float diagSize);

    const char* m_refName   = "EARTH";
    const char* m_frameName = "ECLIPJ2000";  // fallback when m_frames is empty

    // REF input overlay (slot 0)
    bool        m_refInputActive = false;
    char        m_refInputBuf[32]{};
    std::string m_pendingRef;

    // FRM cycling (slot 1)
    std::vector<MFDFrame> m_frames;
    int                   m_frameIdx = 0;

    // TGT cycling (slot 2)
    std::vector<std::string> m_tgtNames;  // display names; index matches spacecraft[]
    int                      m_tgtIdx = -1;  // -1 = none

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

    // --- 3D orbit geometry in the current frame (updated by update()) ---
    // periDir: unit vector toward periapsis
    // normDir: unit orbit normal (= h/|h|)
    // qDir:    unit vector 90° ahead of periapsis in the orbit plane (= normDir × periDir)
    // shipDir: unit vector from body toward spacecraft (for circular orbits)
    glm::dvec3 m_periDir3D {1.0, 0.0, 0.0};
    glm::dvec3 m_normDir3D {0.0, 0.0, 1.0};
    glm::dvec3 m_qDir3D    {0.0, 1.0, 0.0};
    glm::dvec3 m_shipDir3D {1.0, 0.0, 0.0};

    // --- Diagram 3D view ---
    // Identity = looking from frame +Z (top-down on reference plane).
    // Right-drag rotates; double-right-click resets.
    glm::dmat3 m_diagViewRot {1.0};

    // --- Target orbit (optional) ---
    bool   m_hasTgt      = false;
    double m_tgtAltKm    = 0.0;
    double m_tgtVelKms   = 0.0;
    double m_tgtApoKm    = 0.0;
    double m_tgtPerKm    = 0.0;
    double m_tgtIncDeg   = 0.0;
    double m_tgtRaanDeg  = 0.0;
    double m_tgtEccen    = 0.0;
    double m_tgtArgpeDeg = 0.0;
    double m_tgtTrueDeg  = 0.0;
    bool   m_tgtTrueValid = false;
    double m_tgtSmaKm    = 0.0;
    double m_tgtPeriodMin= -1.0;
    bool   m_tgtHyp      = false;
    double m_tgtBodyRadKm= 0.0;
    // 3D geometry for target orbit diagram
    glm::dvec3 m_tgtPeriDir {1.0, 0.0, 0.0};
    glm::dvec3 m_tgtNormDir {0.0, 0.0, 1.0};
    glm::dvec3 m_tgtQDir    {0.0, 1.0, 0.0};
    glm::dvec3 m_tgtShipDir {1.0, 0.0, 0.0};
};
