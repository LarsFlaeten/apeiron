#include "OBCEventQueue.h"
#include "VoiceAnnouncer.h"

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
uint64_t OBCEventQueue::schedule(const std::string& name, double eventET,
                                  bool countdown10s)
{
    // Replace existing event with the same name.
    for (auto& ev : m_events) {
        if (ev.name == name) {
            ev.eventET        = eventET;
            ev.countdown10s   = countdown10s;
            ev.announced10min = false;
            ev.announced5min  = false;
            ev.announced1min  = false;
            ev.announced10s   = false;
            ev.announced0     = false;
            return ev.id;
        }
    }
    ScheduledEvent ev;
    ev.id           = m_nextId++;
    ev.name         = name;
    ev.eventET      = eventET;
    ev.countdown10s = countdown10s;
    m_events.push_back(ev);
    return ev.id;
}

// ---------------------------------------------------------------------------
void OBCEventQueue::cancel(uint64_t id)
{
    m_events.erase(
        std::remove_if(m_events.begin(), m_events.end(),
            [id](const ScheduledEvent& e) { return e.id == id; }),
        m_events.end());
}

// ---------------------------------------------------------------------------
void OBCEventQueue::cancelByName(const std::string& name)
{
    m_events.erase(
        std::remove_if(m_events.begin(), m_events.end(),
            [&name](const ScheduledEvent& e) { return e.name == name; }),
        m_events.end());
}

// ---------------------------------------------------------------------------
void OBCEventQueue::tick(double currentET, double frameDtReal,
                         double& simSpeedTarget, double& simSecondsPerRealSecond)
{
    constexpr double k10min = 600.0;
    constexpr double k5min  = 300.0;
    constexpr double k1min  =  60.0;
    constexpr double k30s   =  30.0;

    for (auto& ev : m_events) {
        const double dt = ev.eventET - currentET;  // positive = future

        // ---- Approach caps -----------------------------------------------
        // At high warp a single frame can advance many minutes of sim time,
        // skipping every deceleration threshold in one step.  For each
        // threshold T, if dt > T we cap speed so the next frame's step is
        // at most (dt − T) seconds — i.e. we land at the boundary, not past
        // it.  The threshold caps below then take over from that point.
        if (dt > 0.0 && frameDtReal > 1e-6) {
            // Approach to the 5-min cap boundary (most critical at high warp).
            // Floor at 100 so we never clamp below the threshold cap that fires next frame.
            if (dt > k5min) {
                const double maxSpeed = std::max((dt - k5min) / frameDtReal, 100.0);
                simSpeedTarget          = std::min(simSpeedTarget,          maxSpeed);
                simSecondsPerRealSecond = std::min(simSecondsPerRealSecond, maxSpeed);
            }
            // Approach to the 1-min cap boundary (relevant from ×100).
            // Floor at 10 so we never clamp below the threshold cap that fires next frame.
            if (dt > k1min) {
                const double maxSpeed = std::max((dt - k1min) / frameDtReal, 10.0);
                simSpeedTarget          = std::min(simSpeedTarget,          maxSpeed);
                simSecondsPerRealSecond = std::min(simSecondsPerRealSecond, maxSpeed);
            }
        }

        // ---- Threshold caps (clamp DOWN both target and current speed) ----
        // Clamping simSecondsPerRealSecond immediately prevents large time steps
        // from flying past the threshold before the smooth interpolation responds.
        if (dt > 0.0 && dt <= k5min) {
            simSpeedTarget          = std::min(simSpeedTarget,          100.0);
            simSecondsPerRealSecond = std::min(simSecondsPerRealSecond, 100.0);
        }
        if (dt > 0.0 && dt <= k1min) {
            simSpeedTarget          = std::min(simSpeedTarget,           10.0);
            simSecondsPerRealSecond = std::min(simSecondsPerRealSecond,  10.0);
        }
        if (dt > 0.0 && dt <= k30s) {
            simSpeedTarget          = std::min(simSpeedTarget,            1.0);
            simSecondsPerRealSecond = std::min(simSecondsPerRealSecond,   1.0);
        }

        constexpr double k10s = 10.0;

        // ---- Silent fast-forward for events already in the past on load ----
        if (dt < 0.0 && !ev.announced10min) {
            ev.announced10min = true;
            ev.announced5min  = true;
            ev.announced1min  = true;
            ev.announced10s   = true;
            ev.announced0     = true;
            continue;   // let the purge below remove it
        }

        // ---- Voice callouts (fire once each) ----
        if (!ev.announced10min && dt <= k10min && dt > 0.0) {
            obc::speak(ev.name + ", T minus 10 minutes");
            ev.announced10min = true;
        }
        if (!ev.announced5min && dt <= k5min && dt > 0.0) {
            obc::warn(ev.name + ", T minus 5 minutes. Reducing time acceleration.");
            ev.announced5min = true;
        }
        if (!ev.announced1min && dt <= k1min && dt > 0.0) {
            obc::warn(ev.name + ", T minus 1 minute.");
            ev.announced1min = true;
        }
        // T-10 s callout — only for events that opted in (IGN, MECO).
        // Uses speakImmediate so it cuts through any queued speech.
        if (ev.countdown10s && !ev.announced10s && dt <= k10s && dt > 0.0) {
            obc::speakImmediate(ev.name + ", T minus 10.");
            ev.announced10s = true;
        }
        if (!ev.announced0 && dt <= 0.0 && dt > -30.0) {
            obc::speakImmediate(ev.name + ".");
            ev.announced0 = true;
        }
    }

    // Remove events more than 30 s in the past.
    m_events.erase(
        std::remove_if(m_events.begin(), m_events.end(),
            [currentET](const ScheduledEvent& e) {
                return (currentET - e.eventET) > 30.0;
            }),
        m_events.end());
}

// ---------------------------------------------------------------------------
void OBCEventQueue::restoreEvents(std::vector<ScheduledEvent> evts)
{
    m_events = std::move(evts);
    // Re-assign sequential ids so cancel() works after a load.
    for (auto& ev : m_events)
        ev.id = m_nextId++;
}
