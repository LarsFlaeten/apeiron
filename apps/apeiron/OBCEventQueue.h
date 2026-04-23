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
    std::string name;                 // e.g. "MCC-1", "MOI"
    double      eventET      = 0.0;   // SPICE ET of the event

    // One-shot announcement guards — preserved across save/load.
    bool announced10min = false;
    bool announced5min  = false;
    bool announced1min  = false;
    bool announced0     = false;
};

class OBCEventQueue {
public:
    // Schedule a new event.  Returns a non-zero id usable for cancel().
    // If an event with the same name already exists it is replaced in-place
    // (announcement flags are reset), preventing duplicate callouts if a
    // plan is reconfirmed.
    uint64_t schedule(const std::string& name, double eventET);

    // Cancel by id (no-op if not found).
    void cancel(uint64_t id);

    // Cancel by name (no-op if not found).
    void cancelByName(const std::string& name);

    // Called once per frame from main.cpp, after currentEt is computed.
    //   currentET      — simulation SPICE ET this frame
    //   simSpeedTarget — IN/OUT: may be reduced to enforce warp caps
    void tick(double currentET, double& simSpeedTarget);

    // Read-only access for SimSave serialisation.
    const std::vector<ScheduledEvent>& events() const { return m_events; }

    // Restore a saved event list (ids are re-assigned sequentially).
    void restoreEvents(std::vector<ScheduledEvent> evts);

private:
    std::vector<ScheduledEvent> m_events;
    uint64_t m_nextId = 1;
};
