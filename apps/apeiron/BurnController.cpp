#include "BurnController.h"
#include "OBCEventQueue.h"
#include "VoiceAnnouncer.h"
#include "apeiron/spacecraft/Autopilot.h"

#include <glm/glm.hpp>

static constexpr double kPreIgnLead = 60.0;   // seconds before ignition to enter PreIgnition
static constexpr double kIgnAbort   = 30.0;   // seconds past planned ignition before abort

// ---------------------------------------------------------------------------
void BurnController::arm(const BurnPlan& plan,
                         spacecraft::Autopilot* ap,
                         OBCEventQueue* eq)
{
    m_plan        = plan;
    m_autopilot   = ap;
    m_evtQueue    = eq;
    m_mecoEventId = 0;
    m_phase       = BurnPhase::Armed;

    // Schedule ignition event (countdown10s = true for T-10 s callout).
    if (eq)
        eq->schedule(plan.name + "-IGN", plan.ignitionET, /* countdown10s= */ true);

    // Slew to burn attitude immediately.
    if (ap)
        ap->mode = plan.retrogradeBurn ? spacecraft::AutopilotMode::Retrograde
                                       : spacecraft::AutopilotMode::Prograde;
}

// ---------------------------------------------------------------------------
void BurnController::disarm()
{
    if (m_evtQueue) {
        m_evtQueue->cancelByName(m_plan.name + "-IGN");
        if (m_mecoEventId) {
            m_evtQueue->cancel(m_mecoEventId);
            m_mecoEventId = 0;
        }
    }
    m_phase = BurnPhase::Idle;
    m_plan  = {};
    // Leave autopilot mode unchanged — caller decides what to do next.
}

// ---------------------------------------------------------------------------
void BurnController::tick(double currentET,
                          const glm::dvec3& shipR,
                          const glm::dvec3& shipV)
{
    switch (m_phase) {

    case BurnPhase::Idle:
    case BurnPhase::Complete:
        return;

    // -----------------------------------------------------------------------
    case BurnPhase::Armed:
        // At T-60 s: enter PreIgnition — re-enforce burn attitude and arm the
        // attitude check so any time-accel interruption gets corrected.
        if (currentET >= m_plan.ignitionET - kPreIgnLead) {
            if (m_autopilot)
                m_autopilot->mode = m_plan.retrogradeBurn
                    ? spacecraft::AutopilotMode::Retrograde
                    : spacecraft::AutopilotMode::Prograde;
            m_phase = BurnPhase::PreIgnition;
        }
        return;

    // -----------------------------------------------------------------------
    case BurnPhase::PreIgnition: {
        // Continuously re-engage burn attitude in case the user knocked it off.
        const auto burnAtt = m_plan.retrogradeBurn
            ? spacecraft::AutopilotMode::Retrograde
            : spacecraft::AutopilotMode::Prograde;
        if (m_autopilot && m_autopilot->mode != burnAtt)
            m_autopilot->mode = burnAtt;

        if (currentET >= m_plan.ignitionET) {
            const bool settled = !m_autopilot || !m_autopilot->inLargeSlew;
            if (settled) {
                // Fire.
                m_phase = BurnPhase::Executing;
                // Schedule estimated MECO for callouts (countdown10s).
                // No hard time cap — MECO fires on C3 condition, not the clock.
                if (m_evtQueue && m_plan.burnDuration > 0.0) {
                    const double mecoET = m_plan.ignitionET + m_plan.burnDuration;
                    m_mecoEventId = m_evtQueue->schedule(
                        "MECO", mecoET, /* countdown10s= */ true);
                }
            } else if (currentET > m_plan.ignitionET + kIgnAbort) {
                // Attitude never settled — abort.
                obc::warn(m_plan.name + " ABORTED — attitude not settled.");
                if (m_evtQueue)
                    m_evtQueue->cancelByName(m_plan.name + "-IGN");
                m_phase = BurnPhase::Idle;
                m_plan  = {};
            }
            // else: still within the abort window, waiting for attitude to settle
        }
        return;
    }

    // -----------------------------------------------------------------------
    case BurnPhase::Executing: {
        const double r = glm::length(shipR);
        if (r < 1.0 || m_plan.depBodyMu <= 0.0) return;

        const double c3Now = glm::dot(shipV, shipV) - 2.0 * m_plan.depBodyMu / r;
        // Prograde (TMI): fire until energy rises above threshold (escape).
        // Retrograde (MOI): fire until energy drops below threshold (capture).
        const bool cutoff = m_plan.retrogradeBurn ? (c3Now <= m_plan.c3Required)
                                                   : (c3Now >= m_plan.c3Required);
        if (cutoff) {
            m_phase = BurnPhase::Complete;

            // Cancel the estimated MECO event and call out the real one.
            if (m_evtQueue && m_mecoEventId) {
                m_evtQueue->cancel(m_mecoEventId);
                m_mecoEventId = 0;
            }
            obc::speakImmediate("MECO.");

            if (m_autopilot)
                m_autopilot->mode = spacecraft::AutopilotMode::Killrot;
        }
        return;
    }

    } // switch
}
