#pragma once

#include <glm/glm.hpp>
#include <string>

namespace spacecraft { class Autopilot; }
class OBCEventQueue;

// ---------------------------------------------------------------------------
// BurnPhase — lifecycle of a single automated burn.
// ---------------------------------------------------------------------------
enum class BurnPhase {
    Idle,         // no plan loaded
    Armed,        // plan loaded, prograde attitude engaged, counting down to ignition
    PreIgnition,  // T-60 s: prograde re-enforced, attitude check before engine start
    Executing,    // engine firing; monitoring C3 for cutoff
    Complete,     // cutoff reached; engine off, Killrot engaged
};

// ---------------------------------------------------------------------------
// BurnPlan — parameters for one automated engine burn.
// Created by TransferMFD and handed to BurnController::arm().
// ---------------------------------------------------------------------------
struct BurnPlan {
    std::string name;             // "TMI" — used as OBC event name suffix
    double      ignitionET   = 0.0;  // absolute ET for engine start
    double      c3Required   = 0.0;  // km²/s²  — cutoff when c3Now >= this
    double      depBodyMu    = 0.0;  // km³/s²  — for live C3 = v²-2μ/r
    double      dvMagnitude  = 0.0;  // km/s    (display / guard only)
    double      burnDuration = 0.0;  // seconds (display / guard only)
};

// ---------------------------------------------------------------------------
// BurnController — sequences a single prograde burn.
//
// ARM:      arm() slews autopilot to Prograde immediately and schedules the
//           "<name>-IGN" OBC event so time-accel slows automatically.
// IGNITION: when currentET >= ignitionET the controller requests main engine.
// CUTOFF:   when C3 = v² - 2μ/r reaches c3Required the engine is stopped and
//           the autopilot is switched to Killrot.
//
// Call tick() every frame with departure-body-centric ship state.
// Read requestMainEngine() and OR it into the main engine demand alongside
// the SPACEBAR key.
// ---------------------------------------------------------------------------
class BurnController {
public:
    void arm   (const BurnPlan& plan, spacecraft::Autopilot* ap, OBCEventQueue* eq);
    void disarm();

    // Per-frame update. shipR/V must be departure-body-centric (km / km·s⁻¹).
    void tick(double currentET, const glm::dvec3& shipR, const glm::dvec3& shipV);

    bool            requestMainEngine() const { return m_phase == BurnPhase::Executing; }
    BurnPhase       phase()             const { return m_phase; }
    const BurnPlan& plan()              const { return m_plan; }
    double          timeToIgnition(double et) const { return m_plan.ignitionET - et; }

    // Returns 10× while the engine is firing (attitude controller is stable
    // at that rate), or 0.0 when no cap is needed.
    double maxSimSpeed() const { return (m_phase == BurnPhase::Executing) ? 10.0 : 0.0; }

private:
    BurnPhase              m_phase       = BurnPhase::Idle;
    BurnPlan               m_plan;
    spacecraft::Autopilot* m_autopilot   = nullptr;
    OBCEventQueue*         m_evtQueue    = nullptr;
    uint64_t               m_mecoEventId = 0;
};
