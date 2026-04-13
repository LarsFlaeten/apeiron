#pragma once

#include <imgui.h>
#include <glm/glm.hpp>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// OrbitDiagram — reusable orbit visualisation widget.
//
// Usage pattern (each frame):
//   OrbitDiagram diag;
//   diag.addOrbit(r, v, mu, colour, "label");
//   diag.addMarker(pos, colour, "E");
//   diag.addArc(points, colour, thickness);
//   diag.addArrow(origin, direction, length, colour);
//   diag.setCentralBody(radiusKm, colour);
//   diag.render(dl, origin, size, &viewRot);   // viewRot persists in caller
//
// Coordinate system: orbits are specified in whatever 3D frame the caller
// uses (e.g. J2000 ecliptic).  The view rotation matrix (3×3, right-drag
// to tumble) maps that frame to screen space.  Default = looking down +Z
// (reference plane fills screen).
//
// All distances in km.  Works for elliptic and hyperbolic orbits.
// ---------------------------------------------------------------------------

struct OrbitDiagram {

    // ---- Input data types ------------------------------------------------

    struct Orbit {
        glm::dvec3  r;          // position at any point on orbit (km)
        glm::dvec3  v;          // velocity at that point (km/s)
        double      mu;         // central body GM (km³/s²)
        ImU32       colour;
        std::string label;      // drawn near periapsis; empty = no label
        bool        showApses   = true;   // draw apse marks + line
        bool        showCurrent = false;  // draw dot at r
        int         segments    = 120;
    };

    struct Marker {
        glm::dvec3  pos;        // 3D position (km)
        ImU32       colour;
        std::string label;
        float       radius = 3.5f;
    };

    struct Arc {
        std::vector<glm::dvec3> pts;   // polyline in 3D (km)
        ImU32       colour;
        float       thickness = 1.5f;
    };

    struct Arrow {
        glm::dvec3 origin;    // km
        glm::dvec3 dir;       // unit vector (direction only)
        float      screenLen; // pixels
        ImU32      colour;
        float      thickness = 1.5f;
    };

    struct CentralBody {
        double radiusKm  = 0.0;    // 0 = auto-scale dot
        ImU32  rimColour = IM_COL32(220, 50, 50, 200);
        ImU32  axisColour= IM_COL32(220, 50, 50, 130);
        bool   drawAxes  = true;
    };

    // ---- Builder API -----------------------------------------------------

    void addOrbit (const Orbit& o)  { orbits.push_back(o); }
    void addMarker(const Marker& m) { markers.push_back(m); }
    void addArc   (const Arc& a)    { arcs.push_back(a); }
    void addArrow (const Arrow& a)  { arrows.push_back(a); }
    void setCentralBody(const CentralBody& b) { centralBody = b; hasCentralBody = true; }

    // Convenience overloads.
    void addOrbit(const glm::dvec3& r, const glm::dvec3& v, double mu,
                  ImU32 col, const std::string& label = "",
                  bool showApses = true, bool showCurrent = false) {
        orbits.push_back({r, v, mu, col, label, showApses, showCurrent});
    }
    void addMarker(const glm::dvec3& pos, ImU32 col, const std::string& label = "") {
        markers.push_back({pos, col, label});
    }
    void addArrow(const glm::dvec3& origin, const glm::dvec3& dir,
                  float screenLen, ImU32 col) {
        arrows.push_back({origin, dir, screenLen, col});
    }

    // ---- Render ----------------------------------------------------------
    // viewRot: caller owns this (persists between frames for interactive
    // rotation).  Pass nullptr for a fixed top-down ecliptic view.
    // Right-drag inside the rect rotates; double-right-click resets.
    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                glm::dmat3* viewRot = nullptr) const;

    // ---- Data ------------------------------------------------------------
    std::vector<Orbit>  orbits;
    std::vector<Marker> markers;
    std::vector<Arc>    arcs;
    std::vector<Arrow>  arrows;
    CentralBody         centralBody;
    bool                hasCentralBody = false;
};
