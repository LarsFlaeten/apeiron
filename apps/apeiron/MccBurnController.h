#pragma once

enum class MccBurnPhase { Idle, Slewing, Burning, Done };

// ---------------------------------------------------------------------------
// MccBurnController
//
// Simple state machine for executing an MCC burn autonomously.
//
// Slewing:  waits until attitude error to MCC direction < kAttTolRad
// Burning:  fires main engine (coarse) or aux thrusters (fine) until
//           |MCC dV| drops below the mode threshold
// Done:     tick() returns true once; caller engages Killrot + calls consumeDone()
//
// Coarse threshold: 1.0 m/s  — main engine
// Fine   threshold: 0.1 m/s  — aux thrusters
// ---------------------------------------------------------------------------
struct MccBurnController {
    MccBurnPhase phase  = MccBurnPhase::Idle;
    bool         coarse = true;

    static constexpr double kCoarseThreshMs = 1.0;
    static constexpr double kFineThreshMs   = 0.1;
    static constexpr double kAttTolRad      = 0.035;  // ~2°

    void arm(bool isCoarse) { coarse = isCoarse; phase = MccBurnPhase::Slewing; }
    void disarm()           { phase  = MccBurnPhase::Idle; }
    bool active()     const { return phase != MccBurnPhase::Idle; }

    // Per-frame tick.  Returns true on the frame the Done state is entered.
    // Caller must engage Killrot and call consumeDone() when true is returned.
    bool tick(double mccDvMs, double attErrorRad) {
        const double thr = coarse ? kCoarseThreshMs : kFineThreshMs;
        switch (phase) {
        case MccBurnPhase::Idle: break;
        case MccBurnPhase::Slewing:
            if (attErrorRad < kAttTolRad)
                phase = (mccDvMs > thr) ? MccBurnPhase::Burning : MccBurnPhase::Done;
            break;
        case MccBurnPhase::Burning:
            if (attErrorRad >= kAttTolRad)
                phase = MccBurnPhase::Slewing;  // attitude drifted — re-slew before resuming
            else if (mccDvMs <= thr)
                phase = MccBurnPhase::Done;
            break;
        case MccBurnPhase::Done: break;
        }
        return phase == MccBurnPhase::Done;
    }

    bool requestMainEngine() const { return  coarse && phase == MccBurnPhase::Burning; }
    bool requestAuxEngine()  const { return !coarse && phase == MccBurnPhase::Burning; }
    void consumeDone()             { if (phase == MccBurnPhase::Done) phase = MccBurnPhase::Idle; }
};
