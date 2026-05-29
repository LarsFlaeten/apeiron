#include "apeiron/spacecraft/PhysicsWorld.h"

#include "astro/SpiceCore.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace spacecraft {

// ---------------------------------------------------------------------------
// SpiceEphemeris
// ---------------------------------------------------------------------------

astro::PosState SpiceEphemeris::getState(int tgtNaif,
                                          const astro::EphemerisTime& et) const
{
    static const astro::ReferenceFrame kEclip =
        astro::ReferenceFrame::createEclipJ2000();
    astro::PosState s;
    astro::Spice().getRelativeGeometricState(tgtNaif, 399, et, s, kEclip);
    return s;
}

// ---------------------------------------------------------------------------
// PhysicsWorld — construction
// ---------------------------------------------------------------------------

PhysicsWorld::PhysicsWorld(std::vector<BodySpec> bodySpecs,
                           const std::string&   startBodyName,
                           const IEphemeris&    eph)
    : m_dominantBody(startBodyName)
    , m_eph(eph)
    , m_eclip(astro::ReferenceFrame::createEclipJ2000())
{
    // Build GravBody table from specs.
    for (auto& spec : bodySpecs) {
        GravBody gb;
        gb.name     = spec.name;
        gb.naifId   = spec.naifId;
        gb.gm       = spec.gm;
        gb.radiusKm = spec.radiusKm;
        gb.soiKm    = 0.0;
        m_bodies.push_back(gb);
    }

    // Compute Hill-sphere (SOI) radii once at startup.
    // r_SOI ≈ a * (GM_body / GM_parent)^(2/5)
    // Parent is Earth for natural satellites (body inside Earth's SOI),
    // Sun for everything else.
    const astro::EphemerisTime t0(0.0);  // J2000 — good enough for static SOI

    double gmSun   = 0.0;
    double gmEarth = 0.0;
    for (const auto& gb : m_bodies) {
        if (gb.name == "SUN")   gmSun   = gb.gm;
        if (gb.name == "EARTH") gmEarth = gb.gm;
    }

    // Sun position relative to Earth at J2000 (used for heliocentric distances).
    glm::dvec3 sunEarthPos(0.0);
    if (gmSun > 0.0) {
        try {
            astro::PosState s = m_eph.getState(10, t0);  // Sun NAIF ID = 10
            sunEarthPos = glm::dvec3(s.r.x, s.r.y, s.r.z);
        } catch (...) {}
    }

    // Earth SOI from Sun — used to classify natural satellites vs planets.
    const double sunEarthDist = glm::length(sunEarthPos);
    const double earthSoiKm = (gmSun > 0.0 && gmEarth > 0.0 && sunEarthDist > 0.0)
        ? sunEarthDist * std::pow(gmEarth / gmSun, 2.0 / 5.0)
        : 0.0;

    if (gmSun > 0.0) {
        for (auto& gb : m_bodies) {
            if (gb.name == "SUN" || gb.name == "EARTH" || gb.gm <= 0.0) continue;
            try {
                astro::PosState s = m_eph.getState(gb.naifId, t0);
                const glm::dvec3 bodyEarthPos = glm::dvec3(s.r.x, s.r.y, s.r.z);
                const double distFromEarth    = glm::length(bodyEarthPos);

                double a, parentGm;
                if (earthSoiKm > 0.0 && distFromEarth < earthSoiKm) {
                    // Natural satellite (e.g. Moon): parent is Earth.
                    a        = distFromEarth;
                    parentGm = gmEarth;
                } else {
                    // Planet or distant body: parent is Sun.
                    // Body-Sun distance = |body_earthPos - sun_earthPos|
                    const double distFromSun = glm::length(bodyEarthPos - sunEarthPos);
                    a        = (distFromSun > 0.0) ? distFromSun : distFromEarth;
                    parentGm = gmSun;
                }

                if (a > 0.0)
                    gb.soiKm = a * std::pow(gb.gm / parentGm, 2.0 / 5.0);
            } catch (...) {}
        }
    }

    // Initialise refBody from startBodyName.
    for (const auto& gb : m_bodies) {
        if (gb.name == startBodyName) {
            m_refBody = { gb.name, gb.gm, gb.radiusKm, gb.naifId };
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// findBody / setDominantBody
// ---------------------------------------------------------------------------

const PhysicsWorld::GravBody* PhysicsWorld::findBody(const std::string& name) const
{
    for (const auto& gb : m_bodies)
        if (gb.name == name) return &gb;
    return nullptr;
}

void PhysicsWorld::setDominantBody(const std::string& name)
{
    for (const auto& gb : m_bodies) {
        if (gb.name != name) continue;
        m_dominantBody = name;
        m_refBody = { gb.name, gb.gm, gb.radiusKm, gb.naifId };
        return;
    }
    // Unknown name — leave state unchanged.
}

// ---------------------------------------------------------------------------
// rebuildAttractors (private)
// ---------------------------------------------------------------------------

void PhysicsWorld::rebuildAttractors(const std::vector<Spacecraft*>& sc,
                                     bool             bodyFrame,
                                     const glm::dvec3& dominantPosECI)
{
    for (auto* s : sc) {
        s->clearAttractors();
        for (const auto& gb : m_bodies) {
            astro::Attractor att;
            att.GM = gb.gm;
            if (bodyFrame) {
                const bool isDominant = (gb.name == m_dominantBody);
                // Dominant body pinned at frame origin; perturbers offset by
                // frame start position so sub-step positions are consistent.
                att.p           = isDominant ? glm::dvec3(0.0)
                                             : gb.posECI - dominantPosECI;
                att.tidal       = !isDominant;
                att.tidalRefPos = glm::dvec3(0.0);
            } else {
                att.p           = gb.posECI;
                att.tidal       = (gb.name != "EARTH");
                att.tidalRefPos = glm::dvec3(0.0);
            }
            s->addAttractor(att);
        }
    }
}

// ---------------------------------------------------------------------------
// updateGravity
// ---------------------------------------------------------------------------

PhysicsWorld::SoiEvent PhysicsWorld::updateGravity(
    const std::vector<Spacecraft*>& sc,
    const glm::dvec3&               playerPos,
    const astro::EphemerisTime&     et)
{
    // Fetch geocentric ECLIPJ2000 positions from the ephemeris.
    // Earth is always at the ECI origin.
    for (auto& gb : m_bodies) {
        gb.prevPosECI = gb.posECI;
        if (gb.name == "EARTH") {
            gb.posECI = glm::dvec3(0.0);
        } else {
            try {
                astro::PosState s = m_eph.getState(gb.naifId, et);
                gb.posECI = glm::dvec3(s.r.x, s.r.y, s.r.z);
            } catch (...) {
                // Leave posECI unchanged on SPICE failure.
            }
        }
    }

    // SOI detection: find the smallest SOI containing the player.
    // Use prevPosECI (last frame's end) for consistency with playerPos.
    std::string bestName = "SUN";
    double      bestSoi  = 1e18;
    for (const auto& gb : m_bodies) {
        if (gb.soiKm <= 0.0) continue;  // SUN or uncomputed
        double r = glm::length(gb.prevPosECI - playerPos);
        if (r < gb.soiKm && gb.soiKm < bestSoi) {
            bestSoi  = gb.soiKm;
            bestName = gb.name;
        }
    }

    SoiEvent evt;
    if (bestName != m_dominantBody) {
        m_dominantBody = bestName;
        for (const auto& gb : m_bodies) {
            if (gb.name != m_dominantBody) continue;
            m_refBody  = { gb.name, gb.gm, gb.radiusKm, gb.naifId };
            evt.changed    = true;
            evt.newRefBody = m_refBody;
            break;
        }
    }

    // Rebuild ECI attractors so MFDs and the next integrate() call start fresh.
    rebuildAttractors(sc, false, glm::dvec3(0.0));
    return evt;
}

// ---------------------------------------------------------------------------
// integrate
// ---------------------------------------------------------------------------

void PhysicsWorld::integrate(double                          simDt,
                              const astro::EphemerisTime&     et,
                              const std::vector<Spacecraft*>& sc,
                              StepCallback                    preTick,
                              StepCallback                    postTick)
{
    // Use body-centric frame when dominant body is neither Earth nor Sun.
    // Earth orbit is exact in ECI (Earth is at the origin).
    // Sun-dominant (interplanetary) stays in ECI — precision is adequate.
    const bool useBodyFrame =
        (m_dominantBody != "EARTH" && m_dominantBody != "SUN");

    glm::dvec3 frameR0(0.0), frameV0(0.0);
    glm::dvec3 frameR1(0.0), frameV1(0.0);

    if (useBodyFrame) {
        const double etStart = et.getETValue() - simDt;
        bool ok = false;
        for (const auto& gb : m_bodies) {
            if (gb.name != m_dominantBody) continue;
            try {
                astro::PosState s0 = m_eph.getState(gb.naifId,
                                                     astro::EphemerisTime(etStart));
                astro::PosState s1 = m_eph.getState(gb.naifId, et);
                frameR0 = glm::dvec3(s0.r.x, s0.r.y, s0.r.z);
                frameV0 = glm::dvec3(s0.v.x, s0.v.y, s0.v.z);
                frameR1 = glm::dvec3(s1.r.x, s1.r.y, s1.r.z);
                frameV1 = glm::dvec3(s1.v.x, s1.v.y, s1.v.z);
                ok = true;
            } catch (...) {}
            break;
        }
        if (!ok) {
            // Fall back to ECI if SPICE query failed.
            goto eci_integrate;
        }

        // Shift free spacecraft to body-centric frame.
        for (auto* s : sc) {
            if (s->parentId()) continue;
            s->setPosition(s->position() - frameR0);
            s->setVelocity(s->velocity() - frameV0);
        }

        rebuildAttractors(sc, true, frameR0);
    }

    {
        // Adaptive step: grow step at high warp but enforce minimum orbit
        // sampling when in body-centric mode.
        double physStep = std::max(kPhysStep,
                                   simDt / static_cast<double>(kMaxStepsPerFrame));

        if (useBodyFrame && m_refBody.mu > 0.0) {
            // Player is at index 0 in the caller's vector; pick first free SC as proxy.
            double r0 = 0.0;
            for (auto* s : sc) {
                if (!s->parentId()) { r0 = glm::length(s->position()); break; }
            }
            if (r0 > 1.0) {
                const double T = 2.0 * std::numbers::pi
                               * std::sqrt(r0 * r0 * r0 / m_refBody.mu);
                physStep = std::clamp(T / kMinOrbitSteps, kPhysStep, physStep);
            }
        }

        m_physAccum += simDt;
        while (m_physAccum >= physStep) {
            preTick(physStep);
            for (auto* s : sc)
                if (!s->parentId())
                    s->update(physStep, et);
            postTick(physStep);
            m_physAccum -= physStep;
        }
    }

    if (useBodyFrame) {
        // Shift all spacecraft (including docked children) back to ECI.
        for (auto* s : sc) {
            s->setPosition(s->position() + frameR1);
            s->setVelocity(s->velocity() + frameV1);
        }
        // Update dominant body's posECI to end-of-frame value.
        for (auto& gb : m_bodies) {
            if (gb.name != m_dominantBody) continue;
            gb.posECI = frameR1;
            break;
        }
        // Restore ECI attractors for MFDs and next-frame SOI detection.
        rebuildAttractors(sc, false, glm::dvec3(0.0));
    }
    return;

eci_integrate:
    // Plain ECI integration (fallback from body-centric SPICE failure).
    double physStep = std::max(kPhysStep,
                               simDt / static_cast<double>(kMaxStepsPerFrame));
    m_physAccum += simDt;
    while (m_physAccum >= physStep) {
        preTick(physStep);
        for (auto* s : sc)
            if (!s->parentId())
                s->update(physStep, et);
        postTick(physStep);
        m_physAccum -= physStep;
    }
}

} // namespace spacecraft
