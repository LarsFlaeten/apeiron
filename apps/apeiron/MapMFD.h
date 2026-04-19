#pragma once

#include "MFD.h"

#include <glm/glm.hpp>
#include <vector>
#include <deque>
#include <string>

// ---------------------------------------------------------------------------
// MapMFD — equirectangular ground-track map of the current reference body.
//
// Displays:
//   • Lat/lon grid (every 30°)
//   • Day/night terminator with shadow overlay
//   • Orbit ground track: 2 (or 3) orbits ahead + ~¼-orbit trail
//   • Sub-satellite point (current ship position)
//   • Sub-solar point
//
// Left buttons:
//   0  TRK–  reduce future orbits shown (min 1)
//   1  TRK+  increase future orbits shown (max 3)
// ---------------------------------------------------------------------------
class MapMFD : public MFDApp {
public:
    const char* name() const override { return "MAP"; }

    struct Config {
        double mu            = 398600.4418; // km³/s²  — reference body GM
        double radiusKm      = 6371.0;      // km      — equatorial radius
        double rotRateRadSec = 7.2921e-5;   // rad/s   — positive = prograde spin
    };

    // Called each frame from main.cpp.
    //   pos / vel        : spacecraft in body-centred ECLIPJ2000 (km / km/s)
    //   cfg              : reference body physical constants
    //   inertialToBody   : rotation from ECLIPJ2000 to body-fixed at current ET
    //   sunDirInertial   : unit vector toward Sun in body-centred ECLIPJ2000
    void update(const glm::dvec3& pos,
                const glm::dvec3& vel,
                const Config&      cfg,
                const glm::dmat3&  inertialToBody,
                const glm::dvec3&  sunDirInertial);

    void setRefName(const char* name) { m_refName = name ? name : ""; }

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel (int slot) const override;
    void        onLeft    (int slot) override;

    // ---- panel coordinate helpers (public so free functions in .cpp can use them) ----
    // Project lat/lon [rad] into pixel coords (equirectangular).
    static ImVec2 project(double lat, double lon, ImVec2 origin, ImVec2 size);

    // Convert body-fixed Cartesian to {lat, lon} in radians.
    static glm::dvec2 toLatLon(const glm::dvec3& r);

    // Rotate v around the z-axis by angle [rad].
    static glm::dvec3 rotZ(const glm::dvec3& v, double angle);

private:
    // ---- drawing helpers ----
    // Draw a polyline of lat/lon points (in radians), breaking at date-line wraps.
    void drawTrack(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                   const std::vector<glm::dvec2>& pts, ImU32 col, float thickness) const;

    // Draw the shadow overlay (semi-transparent fill over the dark hemisphere).
    void drawShadow(ImDrawList* dl, ImVec2 origin, ImVec2 size) const;

    // Draw the terminator line.
    void drawTerminator(ImDrawList* dl, ImVec2 origin, ImVec2 size) const;

    // ---- state (updated by update()) ----
    glm::dvec3  m_pos          { 0.0 };
    glm::dvec3  m_vel          { 0.0 };
    Config      m_cfg;
    glm::dmat3  m_inertialToBody { 1.0 };
    glm::dvec3  m_sunDirInertial { 1.0, 0.0, 0.0 };
    bool        m_valid          = false;

    // Derived elements (set by update(), used by render())
    bool        m_isHyp   = false;
    double      m_ecc     = 0.0;
    double      m_sma     = 0.0;    // km; negative for hyperbolic
    double      m_period  = 0.0;    // s;  0 for hyperbolic
    double      m_p       = 0.0;    // semi-latus rectum [km]
    double      m_nuNow   = 0.0;    // current true anomaly [rad]
    double      m_tFromPe = 0.0;    // current time from periapsis [s]
    glm::dvec3  m_periDir { 1.0, 0.0, 0.0 };
    glm::dvec3  m_normDir { 0.0, 0.0, 1.0 };
    glm::dvec3  m_qDir    { 0.0, 1.0, 0.0 };

    // Past track (body-fixed lat/lon [rad]), recorded every ~30 s sim-time
    std::deque<glm::dvec2> m_trail;
    double      m_trailLastET  = -1e30;
    double      m_lastET       = 0.0;
    static constexpr double kTrailIntervalSec = 30.0;

    // Display options
    int         m_showOrbits = 2;   // future orbits to draw (1–3)

    std::string m_refName;
};
