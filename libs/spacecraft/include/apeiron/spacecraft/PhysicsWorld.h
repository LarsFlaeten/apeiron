#pragma once

#include "apeiron/spacecraft/Spacecraft.h"

#include "astro/ReferenceFrame.h"
#include "astro/State.h"
#include "astro/Time.h"

#include <glm/glm.hpp>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace spacecraft {

// ---------------------------------------------------------------------------
// IEphemeris — injectable SPICE-position provider.
//
// Production: SpiceEphemeris (wraps astro::Spice()).
// Tests:      FakeEphemeris (returns canned PosState values, no kernels needed).
// ---------------------------------------------------------------------------
struct IEphemeris {
    virtual ~IEphemeris() = default;

    // Geocentric (obs = Earth / NAIF 399) ECLIPJ2000 state of tgtNaif.
    // Throws on failure.
    virtual astro::PosState getState(int tgtNaif,
                                     const astro::EphemerisTime& et) const = 0;
};

// ---------------------------------------------------------------------------
// PhysicsWorld
//
// Owns the gravity-body table and drives:
//   1. Per-frame SPICE position updates + SOI detection  (updateGravity)
//   2. Attractor assembly on all Spacecraft              (updateGravity)
//   3. Body-centric frame shift + fixed-step integration (integrate)
//
// The caller owns all Spacecraft objects.  PhysicsWorld holds raw pointers
// only for the duration of each method call; it stores nothing between calls.
//
// All positions are Earth-centred ECLIPJ2000 (km / km·s⁻¹) throughout.
// ---------------------------------------------------------------------------
class PhysicsWorld {
public:
    // -----------------------------------------------------------------------
    // Types
    // -----------------------------------------------------------------------

    struct GravBody {
        std::string name;       // upper-case NAIF name, e.g. "MOON"
        int         naifId;
        double      gm;         // km³/s²
        double      radiusKm;
        double      soiKm;      // Hill-sphere radius; 0 = no finite SOI (SUN)
        glm::dvec3  posECI;     // geocentric ECLIPJ2000, km — updated each frame
        glm::dvec3  prevPosECI; // posECI from previous frame (for SOI detection)
    };

    struct RefBody {
        std::string name;
        double      mu       = 0.0;
        double      radiusKm = 0.0;
        int         naifId   = 399;
    };

    // Minimal description of a body needed at construction time.
    struct BodySpec {
        std::string name;
        int         naifId;
        double      gm;       // km³/s²
        double      radiusKm;
    };

    // Returned by updateGravity(); carry refBody into the caller for MFD update.
    struct SoiEvent {
        bool    changed = false;
        RefBody newRefBody;     // valid only when changed == true
    };

    // Callbacks fired once per physics sub-step inside integrate().
    using StepCallback = std::function<void(double dtSec)>;

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    // bodySpecs:      all gravity bodies for this scenario (SUN, EARTH, MOON, …).
    // startBodyName:  the body the player starts inside the SOI of (e.g. "EARTH").
    // eph:            non-owning reference; must outlive this object.
    PhysicsWorld(std::vector<BodySpec> bodySpecs,
                 const std::string&   startBodyName,
                 const IEphemeris&    eph);

    // -----------------------------------------------------------------------
    // Per-frame update
    // -----------------------------------------------------------------------

    // Fetch geocentric ECLIPJ2000 positions from the ephemeris, detect SOI
    // transitions, and rebuild ECI attractors on all spacecraft.
    //
    // playerPos:  ECI position (km) used for SOI detection.
    // et:         simulation ET for this frame (end-of-frame).
    // sc:         all active spacecraft (non-owning).
    SoiEvent updateGravity(const std::vector<Spacecraft*>& sc,
                           const glm::dvec3&               playerPos,
                           const astro::EphemerisTime&     et);

    // Run fixed-step physics integration for simDt seconds of simulation time.
    //
    // preTick:  called before each sub-step (e.g. DockingConstraint::update).
    // postTick: called after each sub-step (e.g. DockingConstraint::enforcePostStep).
    //
    // Internally switches to dominant-body-centric frame when applicable,
    // and restores ECI attractors before returning.
    void integrate(double                          simDt,
                   const astro::EphemerisTime&     et,
                   const std::vector<Spacecraft*>& sc,
                   StepCallback                    preTick,
                   StepCallback                    postTick);

    // -----------------------------------------------------------------------
    // State accessors
    // -----------------------------------------------------------------------

    const std::string&           dominantBodyName() const { return m_dominantBody; }
    const RefBody&               refBody()          const { return m_refBody; }
    const std::vector<GravBody>& gravBodies()       const { return m_bodies; }
    double                       physAccum()        const { return m_physAccum; }

    const GravBody* findBody(const std::string& name) const;

    // Override dominant body (used on save-game restore and manual MFD change).
    void setDominantBody(const std::string& name);

    // -----------------------------------------------------------------------
    // Constants
    // -----------------------------------------------------------------------
    static constexpr double kPhysStep         = 0.01;  // minimum sub-step, seconds
    static constexpr int    kMaxStepsPerFrame = 100;
    static constexpr double kMinOrbitSteps    = 100.0;

private:
    void rebuildAttractors(const std::vector<Spacecraft*>& sc,
                           bool                            bodyFrame,
                           const glm::dvec3&               dominantPosECI);

    std::vector<GravBody>  m_bodies;
    std::string            m_dominantBody;
    RefBody                m_refBody;
    double                 m_physAccum = 0.0;
    const IEphemeris&      m_eph;
    astro::ReferenceFrame  m_eclip;
};

// ---------------------------------------------------------------------------
// SpiceEphemeris — production IEphemeris backed by astro::Spice().
// ---------------------------------------------------------------------------
class SpiceEphemeris final : public IEphemeris {
public:
    astro::PosState getState(int tgtNaif,
                             const astro::EphemerisTime& et) const override;
};

} // namespace spacecraft
