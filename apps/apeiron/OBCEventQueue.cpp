#include "OBCEventQueue.h"
#include "VoiceAnnouncer.h"

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
uint64_t OBCEventQueue::schedule(const std::string& name, double eventET)
{
    // Replace existing event with the same name.
    for (auto& ev : m_events) {
        if (ev.name == name) {
            ev.eventET        = eventET;
            ev.announced10min = false;
            ev.announced5min  = false;
            ev.announced1min  = false;
            ev.announced0     = false;
            return ev.id;
        }
    }
    ScheduledEvent ev;
    ev.id      = m_nextId++;
    ev.name    = name;
    ev.eventET = eventET;
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
void OBCEventQueue::tick(double currentET, double& simSpeedTarget)
{
    constexpr double k10min = 600.0;
    constexpr double k5min  = 300.0;
    constexpr double k1min  =  60.0;
    constexpr double k30s   =  30.0;

    for (auto& ev : m_events) {
        const double dt = ev.eventET - currentET;  // positive = future

        // ---- Warp caps (only clamp DOWN, never raise) ----
        if (dt > 0.0 && dt <= k5min)
            simSpeedTarget = std::min(simSpeedTarget, 100.0);
        if (dt > 0.0 && dt <= k1min)
            simSpeedTarget = std::min(simSpeedTarget, 10.0);
        if (dt > 0.0 && dt <= k30s)
            simSpeedTarget = std::min(simSpeedTarget, 1.0);

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
