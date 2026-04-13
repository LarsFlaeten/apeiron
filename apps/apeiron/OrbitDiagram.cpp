#include "OrbitDiagram.h"

#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>
#include <numbers>

static constexpr double kTwoPi = 2.0 * std::numbers::pi;

// ---------------------------------------------------------------------------
// Decompose state vector into orbital geometry.
// Returns false if the orbit cannot be drawn (bad inputs, parabolic edge).
struct OrbGeom {
    bool       valid   = false;
    bool       isHyp   = false;
    double     ecc     = 0.0;
    double     slr     = 0.0;   // semi-latus rectum (km)
    double     sma     = 0.0;   // semi-major axis   (km, negative if hyp)
    glm::dvec3 periDir;         // unit vector toward periapsis
    glm::dvec3 qDir;            // unit vector 90° ahead in orbit plane
    glm::dvec3 normDir;         // unit orbit normal
};

static OrbGeom makeGeom(const glm::dvec3& r, const glm::dvec3& v, double mu)
{
    OrbGeom g;
    if (mu <= 0.0) return g;
    double rMag = glm::length(r);
    if (rMag < 1.0) return g;

    glm::dvec3 h    = glm::cross(r, v);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) return g;

    g.normDir = h / hMag;
    g.slr     = hMag * hMag / mu;

    glm::dvec3 ev  = glm::cross(v, h) / mu - r / rMag;
    g.ecc          = glm::length(ev);

    if (g.ecc > 1e-9)
        g.periDir = glm::normalize(ev);
    else
        g.periDir = glm::normalize(r);

    g.qDir   = glm::cross(g.normDir, g.periDir);
    g.isHyp  = (g.ecc >= 1.0);
    g.sma    = g.isHyp ? -g.slr / (g.ecc * g.ecc - 1.0)
                        :  g.slr / (1.0 - g.ecc * g.ecc);
    g.valid  = true;
    return g;
}

// True anomaly of position r given geometry (radians, range depends on orbit).
static double trueAnom(const OrbGeom& g, const glm::dvec3& r)
{
    double cosnu = glm::dot(g.periDir, glm::normalize(r));
    cosnu = std::clamp(cosnu, -1.0, 1.0);
    double nu = std::acos(cosnu);
    if (glm::dot(glm::cross(g.periDir, r), g.normDir) < 0.0) nu = -nu;
    return nu;
}

// ---------------------------------------------------------------------------
void OrbitDiagram::render(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                          glm::dmat3* viewRot) const
{
    const float pad = 6.0f;
    if (size.x < pad * 2 || size.y < pad * 2) return;

    // ---- Static identity rotation if no viewRot passed ----
    static const glm::dmat3 kIdentity(1.0);
    const glm::dmat3& rot = viewRot ? *viewRot : kIdentity;

    // ---- Mouse interaction (only when viewRot is provided) ----
    if (viewRot) {
        const ImVec2 br = { origin.x + size.x, origin.y + size.y };
        if (ImGui::IsMouseHoveringRect(origin, br)) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                const ImVec2 d  = ImGui::GetIO().MouseDelta;
                const double dx = d.x * 0.006;
                const double dy = d.y * 0.006;
                if (dx != 0.0 || dy != 0.0) {
                    const glm::dmat3 Rx(glm::rotate(glm::dmat4(1.0), dy, {1.0, 0.0, 0.0}));
                    const glm::dmat3 Ry(glm::rotate(glm::dmat4(1.0), dx, {0.0, 1.0, 0.0}));
                    *viewRot = Rx * Ry * *viewRot;
                }
            }
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right))
                *viewRot = glm::dmat3(1.0);
        }
    }

    const float cx = origin.x + size.x * 0.5f;
    const float cy = origin.y + size.y * 0.5f;

    // ---- Auto-scale: find the maximum extent of all objects ----
    double maxExtent = 1.0;   // km
    for (const auto& o : orbits) {
        OrbGeom g = makeGeom(o.r, o.v, o.mu);
        if (!g.valid) continue;
        double apoKm = g.isHyp
            ? std::abs(g.sma) * 2.0
            : std::abs(g.sma) * (1.0 + std::min(g.ecc, 0.99));
        maxExtent = std::max(maxExtent, apoKm);
    }
    for (const auto& m : markers)
        maxExtent = std::max(maxExtent, glm::length(m.pos) * 1.05);
    for (const auto& a : arcs) {
        for (const auto& p : a.pts)
            maxExtent = std::max(maxExtent, glm::length(p) * 1.05);
    }

    const float usable = std::min(size.x, size.y) - 2.0f * pad;
    const double scale = (usable * 0.5) / (maxExtent * 1.05);  // px/km

    // ---- Projection helpers ----
    auto proj = [&](const glm::dvec3& p) -> ImVec2 {
        const glm::dvec3 v = rot * p;
        return { cx + static_cast<float>(v.x * scale),
                 cy - static_cast<float>(v.y * scale) };
    };
    auto viewZ = [&](const glm::dvec3& p) -> double {
        return (rot * p).z;
    };

    // ---- Reference plane ring (very dim, 80% of max extent) ----
    {
        const double rr = maxExtent * 0.8;
        constexpr int kSeg = 64;
        ImVec2 pts[kSeg + 1];
        for (int i = 0; i <= kSeg; ++i) {
            double t = kTwoPi * i / kSeg;
            pts[i] = proj({ rr * std::cos(t), rr * std::sin(t), 0.0 });
        }
        dl->AddPolyline(pts, kSeg + 1, IM_COL32(0, 70, 25, 35),
                        ImDrawFlags_None, 0.5f);
    }

    // ---- Orbits ----
    for (const auto& o : orbits) {
        OrbGeom g = makeGeom(o.r, o.v, o.mu);
        if (!g.valid) continue;

        const int kSeg = o.segments;
        double nu0, nu1;
        if (g.isHyp) {
            double nuInf = std::acos(-1.0 / g.ecc) - 0.04;
            nu0 = -nuInf; nu1 = nuInf;
        } else {
            nu0 = 0.0; nu1 = kTwoPi;
        }

        std::vector<ImVec2> pts(kSeg + 1);
        for (int i = 0; i <= kSeg; ++i) {
            double nu = nu0 + (nu1 - nu0) * i / kSeg;
            double r_ = g.slr / (1.0 + g.ecc * std::cos(nu));
            pts[i] = proj(r_ * (std::cos(nu) * g.periDir + std::sin(nu) * g.qDir));
        }
        dl->AddPolyline(pts.data(), kSeg + 1, o.colour, ImDrawFlags_None, 1.0f);

        // Asymptote lines (hyperbolic).
        if (g.isHyp) {
            double nuInf = std::acos(-1.0 / g.ecc);
            for (int sign : {+1, -1}) {
                glm::dvec3 dir = std::cos(nuInf) * g.periDir
                               + sign * std::sin(nuInf) * g.qDir;
                ImVec2 p0 = {cx, cy};
                ImVec2 p1 = proj(dir * maxExtent * 1.5);
                float dx = p1.x - p0.x, dy = p1.y - p0.y;
                float len = std::sqrt(dx*dx + dy*dy);
                if (len > 0.5f) {
                    float ext = usable * 0.65f;
                    dl->AddLine(p0, { cx + dx/len*ext, cy + dy/len*ext },
                                IM_COL32(180,100,0,70), 0.8f);
                }
            }
        }

        // Apse marks.
        if (o.showApses && g.ecc > 0.005) {
            double rp = g.slr / (1.0 + g.ecc);
            ImVec2 periPt = proj(rp * g.periDir);
            if (!g.isHyp) {
                double ra = g.slr / (1.0 - g.ecc);
                ImVec2 apoPt = proj(-ra * g.periDir);
                dl->AddLine(periPt, apoPt,
                            IM_COL32(0,120,40,60), 0.5f);
                dl->AddCircle(apoPt, 2.5f, o.colour, 0, 1.0f);
            } else {
                dl->AddLine({cx,cy}, periPt, IM_COL32(0,120,40,60), 0.5f);
            }
            dl->AddCircle(periPt, 3.0f, o.colour, 0, 1.2f);
        }

        // Current position dot.
        if (o.showCurrent) {
            ImVec2 shipPt = proj(o.r);
            dl->AddLine({cx, cy}, shipPt, o.colour, 1.0f);
            dl->AddCircleFilled(shipPt, 4.0f, o.colour);
        }

        // Label near periapsis.
        if (!o.label.empty() && g.ecc > 0.005) {
            double rp = g.slr / (1.0 + g.ecc);
            ImVec2 pp = proj(rp * g.periDir);
            dl->AddText({ pp.x + 3, pp.y - 8 }, o.colour, o.label.c_str());
        }
    }

    // ---- Arcs (polylines in 3D space) ----
    for (const auto& a : arcs) {
        if (a.pts.size() < 2) continue;
        std::vector<ImVec2> spts;
        spts.reserve(a.pts.size());
        for (const auto& p : a.pts)
            spts.push_back(proj(p));
        dl->AddPolyline(spts.data(), static_cast<int>(spts.size()),
                        a.colour, ImDrawFlags_None, a.thickness);
    }

    // ---- Central body ----
    if (hasCentralBody) {
        double bR = (centralBody.radiusKm > 0.0)
            ? centralBody.radiusKm
            : maxExtent * 0.025;

        // Equatorial ring (front/back shading).
        constexpr int kBSeg = 48;
        for (int i = 0; i < kBSeg; ++i) {
            double t0 = kTwoPi * i       / kBSeg;
            double t1 = kTwoPi * (i + 1) / kBSeg;
            glm::dvec3 p0 = bR * glm::dvec3(std::cos(t0), std::sin(t0), 0.0);
            glm::dvec3 p1 = bR * glm::dvec3(std::cos(t1), std::sin(t1), 0.0);
            bool front = (viewZ(p0) + viewZ(p1)) >= 0.0;
            dl->AddLine(proj(p0), proj(p1),
                        front ? centralBody.rimColour
                              : IM_COL32(150, 30, 30, 50), 1.0f);
        }

        // Axis stubs (if enabled).
        if (centralBody.drawAxes) {
            auto axEnd = [&](const glm::dvec3& dir) -> ImVec2 {
                glm::dvec3 v = rot * dir;
                float f = static_cast<float>(bR * scale * 1.6);
                return { cx + static_cast<float>(v.x) * f, cy - static_cast<float>(v.y) * f };
            };
            ImVec2 centre = {cx, cy};
            dl->AddLine(centre, axEnd({1,0,0}), centralBody.axisColour, 1.0f);
            dl->AddLine(centre, axEnd({0,1,0}), centralBody.axisColour, 1.0f);
            dl->AddLine(centre, axEnd({0,0,1}), centralBody.axisColour, 1.0f);
        }
    } else {
        // Simple dot for Sun / point mass.
        dl->AddCircleFilled({cx, cy}, 4.0f, IM_COL32(255, 220, 60, 220));
    }

    // ---- Markers ----
    for (const auto& m : markers) {
        ImVec2 sp = proj(m.pos);
        // Dim spoke from centre to marker (50% alpha of marker colour).
        dl->AddLine({cx, cy}, sp, (m.colour & 0x00FFFFFFu) | 0x50000000u, 0.8f);
        dl->AddCircleFilled(sp, m.radius, m.colour);
        if (!m.label.empty())
            dl->AddText({ sp.x + m.radius + 2.0f, sp.y - 5.0f }, m.colour,
                        m.label.c_str());
    }

    // ---- Arrows ----
    for (const auto& a : arrows) {
        ImVec2 from = proj(a.origin);
        glm::dvec3 vd = rot * a.dir;
        ImVec2 to = { from.x + static_cast<float>(vd.x) * a.screenLen,
                      from.y - static_cast<float>(vd.y) * a.screenLen };
        dl->AddLine(from, to, a.colour, a.thickness);
        dl->AddCircleFilled(to, a.thickness + 1.0f, a.colour);
    }
}
