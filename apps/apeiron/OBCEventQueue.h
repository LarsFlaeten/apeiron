#pragma once

#include <string>
#include <vector>
#include <cstdint>

// ---------------------------------------------------------------------------
// OBCEventQueue — scheduled mission-event announcements.
//
// MFDs (and any other subsystem) post named, timed events via schedule().
// tick() is called once per frame; it:
//   • fires voice callouts at T−10 min, T−5 min, T−1 min, and T=0
//   • caps simSpeedTarget to enforce warp-down ramps near the event:
//       T < 5 min  → max 100×
//       T < 1 min  → max  10×
//       T < 30 s   → max   1×
//
// The cap is one-directional: it only reduces simSpeedTarget, never raises
// it, so a user already at lower warp is unaffected.
//
// Events persist through quicksave/load via SimSave (see SimSave.h).
// ---------------------------------------------------------------------------

struct ScheduledEvent {
    uint64_t    id           = 0;     // opaque handle; 0 = invalid
    std::string name;                 // e.g. "TCM-1", "MOI"
    double      eventET      = 0.0;   // SPICE ET of the event

    // One-shot announcement guards — preserved across save/load.
    bool announced10min = false;
    bool announced5min  = false;
    bool announced1min  = false;
    bool announced10s   = false;   // only fires when countdown10s == true
    bool announced0     = false;

    // When true, an extra T-10 s voice callout fires (speakImmediate).
    // Useful for time-critical events like IGN and MECO.
    bool countdown10s   = false;
};

class OBCEventQueue {
public:
    // Schedule a new event.  Returns a non-zero id usable for cancel().
    // If an event with the same name already exists it is replaced in-place.
    // Pass currentET > 0 to pre-mark announcement milestones that have already
    // passed, suppressing spurious callouts when the event is imminent at arm time.
    // Set countdown10s = true to get an extra T-10 s voice callout in addition
    // to the standard T-10 min / T-5 min / T-1 min / T-0 set.
    uint64_t schedule(const std::string& name, double eventET,
                      bool countdown10s = false, double currentET = 0.0);

    // Silently update the ET of an existing event without resetting announcement
    // flags.  Used for continuous ignition-time refinement so callouts track the
    // live ignition estimate rather than the originally-armed estimate.
    // No-op if the event does not exist.
    void updateEventTime(const std::string& name, double newET);

    // Cancel by id (no-op if not found).
    void cancel(uint64_t id);

    // Cancel by name (no-op if not found).
    void cancelByName(const std::string& name);

    // Called once per frame from main.cpp, BEFORE the sim-time advance.
    //   currentET              — pre-advance SPICE ET (previous frame's state)
    //   frameDtReal            — real-time frame duration (seconds), used to compute
    //                            approach caps that prevent skipping thresholds in
    //                            a single large time step at high warp.
    //   simSpeedTarget         — IN/OUT: clamped to enforce warp caps
    //   simSecondsPerRealSecond— IN/OUT: clamped immediately (current frame speed)
    //
    // Two-layer defence:
    //   1. Threshold caps   — hard limits once inside T-5min / T-1min / T-30s windows.
    //   2. Approach caps    — reduce max warp so the NEXT frame's step lands at most
    //                         at the threshold boundary, never past it.  Prevents
    //                         high-warp frames (e.g. 28 min/frame at ×100 000) from
    //                         jumping over the entire deceleration ramp in one step.
    void tick(double currentET, double frameDtReal,
              double& simSpeedTarget, double& simSecondsPerRealSecond);

    // Read-only access for SimSave serialisation.
    const std::vector<ScheduledEvent>& events() const { return m_events; }

    // Restore a saved event list (ids are re-assigned sequentially).
    void restoreEvents(std::vector<ScheduledEvent> evts);

private:
    std::vector<ScheduledEvent> m_events;
    uint64_t m_nextId = 1;
};
