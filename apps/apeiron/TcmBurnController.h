#pragma once

enum class TcmBurnPhase { Idle, Slewing, Burning, Done };

// ---------------------------------------------------------------------------
// TcmBurnController
//
// Simple state machine for executing an TCM burn autonomously.
//
// Slewing:  waits until attitude error to TCM direction < kAttTolRad
// Burning:  fires main engine (coarse) or aux thrusters (fine) until
//           |TCM dV| drops below the mode threshold
// Done:     tick() returns true once; caller engages Killrot + calls consumeDone()
//
// Coarse threshold: 1.0 m/s  — main engine
// Fine   threshold: 0.1 m/s  — aux thrusters
// ---------------------------------------------------------------------------
struct TcmBurnController {
    TcmBurnPhase phase  = TcmBurnPhase::Idle;
    bool         coarse = true;

    static constexpr double kCoarseThreshMs = 1.0;
    static constexpr double kFineThreshMs   = 0.1;
    static constexpr double kAttTolRad      = 0.035;  // ~2°

    void arm(bool isCoarse) { coarse = isCoarse; phase = TcmBurnPhase::Slewing; }
    void disarm()           { phase  = TcmBurnPhase::Idle; }
    bool active()     const { return phase != TcmBurnPhase::Idle; }

    // Per-frame tick.  Returns true on the frame the Done state is entered.
    // Caller must engage Killrot and call consumeDone() when true is returned.
    bool tick(double tcmDvMs, double attErrorRad) {
        const double thr = coarse ? kCoarseThreshMs : kFineThreshMs;
        switch (phase) {
        case TcmBurnPhase::Idle: break;
        case TcmBurnPhase::Slewing:
            if (tcmDvMs <= thr)
                phase = TcmBurnPhase::Done;        // already done — no need to aim first
            else if (attErrorRad < kAttTolRad)
                phase = TcmBurnPhase::Burning;
            break;
        case TcmBurnPhase::Burning:
            if (attErrorRad >= kAttTolRad)
                phase = TcmBurnPhase::Slewing;  // attitude drifted — re-slew before resuming
            else if (tcmDvMs <= thr)
                phase = TcmBurnPhase::Done;
            break;
        case TcmBurnPhase::Done: break;
        }
        return phase == TcmBurnPhase::Done;
    }

    bool requestMainEngine() const { return  coarse && phase == TcmBurnPhase::Burning; }
    bool requestAuxEngine()  const { return !coarse && phase == TcmBurnPhase::Burning; }
    void consumeDone()             { if (phase == TcmBurnPhase::Done) phase = TcmBurnPhase::Idle; }
};
