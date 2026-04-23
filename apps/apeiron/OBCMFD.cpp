#include "OBCMFD.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

// ---- palette ----------------------------------------------------------------
static const ImU32 kObcGreen  = IM_COL32(  0, 210,  75, 210);
static const ImU32 kObcDim    = IM_COL32( 80, 130,  80, 160);
static const ImU32 kObcAmber  = IM_COL32(255, 180,  40, 230);
static const ImU32 kObcBright = IM_COL32(255, 255, 200, 255);
static const ImU32 kObcDiv    = IM_COL32(  0, 120,  40, 120);

// Dot colours: ■ fired vs □ pending
static const ImU32 kDotFired   = IM_COL32(  0, 210,  75, 200);
static const ImU32 kDotPending = IM_COL32( 50, 100,  55, 130);

// ---------------------------------------------------------------------------
// Format a signed countdown in seconds to a "T-D+HH:MM" or "T-HH:MM:SS"
// string.  Negative dt = past event → "T+…"
static void fmtCountdown(double dt, char* buf, std::size_t sz)
{
    const char sign = (dt >= 0.0) ? '-' : '+';
    double abs_dt = std::fabs(dt);

    int days = static_cast<int>(abs_dt / 86400.0);
    int rem  = static_cast<int>(std::fmod(abs_dt, 86400.0));
    int hh   = rem / 3600; rem %= 3600;
    int mm   = rem / 60;
    int ss   = rem % 60;

    if (days > 0)
        std::snprintf(buf, sz, "T%c%dd %02d:%02d", sign, days, hh, mm);
    else
        std::snprintf(buf, sz, "T%c%02d:%02d:%02d", sign, hh, mm, ss);
}

// ---------------------------------------------------------------------------
void OBCMFD::update(const MFDContext& ctx)
{
    m_queue     = ctx.eventQueue;
    m_currentET = ctx.currentEt.getETValue();
}

// ---------------------------------------------------------------------------
const char* OBCMFD::leftLabel(int slot) const
{
    if (slot == 0) return "CLR";
    if (slot == 1) return "SKIP";
    return "";
}

// ---------------------------------------------------------------------------
void OBCMFD::onLeft(int slot)
{
    if (!m_queue) return;

    if (slot == 0) {
        // Remove all events whose Exec announcement has fired (ann0 = true),
        // or which are more than 30 s in the past.
        const auto& evts = m_queue->events();
        std::vector<uint64_t> toCancel;
        for (const auto& ev : evts) {
            double dt = ev.eventET - m_currentET;
            if (ev.announced0 || dt < -30.0)
                toCancel.push_back(ev.id);
        }
        for (uint64_t id : toCancel)
            m_queue->cancel(id);
    }
    else if (slot == 1) {
        // Mark the chronologically next upcoming event as immediately done,
        // then cancel it.  Used when a burn was performed outside the
        // scheduled window (e.g. post-burn scenario loaded from an old save).
        const auto& evts = m_queue->events();
        uint64_t nextId = 0;
        double   nextDt = 1e30;
        for (const auto& ev : evts) {
            double dt = ev.eventET - m_currentET;
            if (dt > 0.0 && dt < nextDt) {
                nextDt = dt;
                nextId = ev.id;
            }
        }
        if (nextId != 0)
            m_queue->cancel(nextId);
    }
}

// ---------------------------------------------------------------------------
void OBCMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const float pad   = 4.0f;
    const float lineH = ImGui::GetTextLineHeight() + 4.0f;

    // ---- header row ---------------------------------------------------------
    float y = origin.y + pad;
    {
        const char* hdr = "EVENT SCHEDULE";
        ImVec2 hsz = ImGui::CalcTextSize(hdr);
        dl->AddText({ origin.x + (size.x - hsz.x) * 0.5f, y }, kObcDim, hdr);
        y += lineH;
        dl->AddLine({ origin.x + pad, y - 2.0f },
                    { origin.x + size.x - pad, y - 2.0f }, kObcDiv);
    }

    // ---- no data cases ------------------------------------------------------
    if (!m_queue) {
        dl->AddText({ origin.x + pad, y }, kObcDim, "No event queue");
        return;
    }

    // Sorted copy by event time.
    auto evts = m_queue->events();
    std::sort(evts.begin(), evts.end(),
              [](const ScheduledEvent& a, const ScheduledEvent& b) {
                  return a.eventET < b.eventET;
              });

    if (evts.empty()) {
        const char* msg = "No events scheduled";
        ImVec2 msz = ImGui::CalcTextSize(msg);
        float my = origin.y + size.y * 0.5f - msz.y * 0.5f;
        dl->AddText({ origin.x + (size.x - msz.x) * 0.5f, my }, kObcDim, msg);
        return;
    }

    // ---- column layout ------------------------------------------------------
    // Content width between the two button columns.
    const float cw = size.x;

    // Time field: "T-DD+HH:MM" = 11 chars max.  Measure once.
    const float timeW = ImGui::CalcTextSize("T-000d 00:00").x + pad;

    // Dot block: 4 dots × (dotSz + gap).
    const float dotSz   = 5.0f;
    const float dotGap  = 5.0f;
    const float dotsW   = 4.0f * dotSz + 3.0f * dotGap;  // ~35 px

    // Name field: whatever is left.
    const float nameW = cw - pad * 2.0f - dotsW - timeW - pad;

    // X positions.
    const float nameX  = origin.x + pad;
    const float dotsX  = nameX + nameW + pad;
    const float timeX  = origin.x + cw - pad - timeW;

    // ---- event rows ---------------------------------------------------------
    for (const auto& ev : evts) {
        if (y + lineH > origin.y + size.y) break;   // clip to panel

        const double dt   = ev.eventET - m_currentET;
        const bool   past = ev.announced0 || dt < -30.0;

        // Row colour.
        ImU32 col;
        if      (past)       col = kObcDim;
        else if (dt < 60.0)  col = kObcBright;
        else if (dt < 300.0) col = kObcAmber;
        else                 col = kObcGreen;

        // Name — truncate to available width.
        {
            char nameBuf[32];
            std::snprintf(nameBuf, sizeof(nameBuf), "%s", ev.name.c_str());
            // Clip text if it overflows the name column.
            const char* end = nameBuf + strlen(nameBuf);
            while (end > nameBuf &&
                   ImGui::CalcTextSize(nameBuf, end).x > nameW)
                --end;
            dl->AddText({ nameX, y }, col, nameBuf, end);
        }

        // Announcement dots: 10m  5m  1m  Exec  (left → right).
        {
            const bool fired[4] = {
                ev.announced10min,
                ev.announced5min,
                ev.announced1min,
                ev.announced0,
            };
            for (int i = 0; i < 4; ++i) {
                float dx = dotsX + i * (dotSz + dotGap);
                float dy = y + (ImGui::GetTextLineHeight() - dotSz) * 0.5f;
                ImVec2 pMin{ dx, dy };
                ImVec2 pMax{ dx + dotSz, dy + dotSz };
                if (fired[i])
                    dl->AddRectFilled(pMin, pMax, kDotFired, 1.0f);
                else
                    dl->AddRect(pMin, pMax, kDotPending, 1.0f);
            }
        }

        // Countdown.
        {
            char tbuf[24];
            fmtCountdown(dt, tbuf, sizeof(tbuf));
            ImVec2 tsz = ImGui::CalcTextSize(tbuf);
            dl->AddText({ origin.x + cw - pad - tsz.x, y }, col, tbuf);
        }

        y += lineH;

        // Thin separator between rows.
        dl->AddLine({ origin.x + pad, y - 2.0f },
                    { origin.x + cw - pad, y - 2.0f },
                    IM_COL32(0, 80, 30, 60));
    }
}
