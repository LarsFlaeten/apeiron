#pragma once

#include "MFD.h"
#include "MFDContext.h"
#include "OBCEventQueue.h"

// ---------------------------------------------------------------------------
// OBCMFD — On-Board Computer status display.
//
// Page 0: Event schedule
//   Lists all scheduled mission events (TMI, MOI, TCM-1 …) sorted by time.
//   Each row shows:
//     • Event name (up to 8 chars)
//     • Four announcement-state indicators (T−10m  T−5m  T−1m  Exec)
//     • Countdown / elapsed  (T−D+HH:MM  or  T+HH:MM:SS)
//   Row colour:
//     Green  — future, > 5 min away
//     Amber  — imminent, 1 – 5 min
//     White  — < 1 min  (about to fire)
//     Dim    — past (Exec flag set, or > 30 s elapsed)
//
// Buttons:
//   Left 0: CLR   remove all past/executed events from the queue
//   Left 1: SKIP  immediately mark the next upcoming event as done and remove it
//                 (use when a burn was performed outside the scheduled window)
// ---------------------------------------------------------------------------
class OBCMFD : public MFDApp {
public:
    const char* name() const override { return "OBC"; }

    // Called once per frame from the consolidated MFD context block.
    void update(const MFDContext& ctx);

    void render(ImDrawList* dl, ImVec2 origin, ImVec2 size) override;

    const char* leftLabel(int slot) const override;
    void        onLeft   (int slot) override;

private:
    OBCEventQueue* m_queue     = nullptr;
    double         m_currentET = 0.0;
};
