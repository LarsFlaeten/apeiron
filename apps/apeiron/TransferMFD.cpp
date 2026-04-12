#include "TransferMFD.h"

#include <astro/SpiceCore.h>

#include <cmath>
#include <cstdio>
#include <algorithm>

static constexpr double kDay = 86400.0;

// ---------------------------------------------------------------------------
// Colour map: t ∈ [0,1]  →  green → yellow → red
// ---------------------------------------------------------------------------
ImU32 TransferMFD::dvColor(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    uint8_t r, g, b;
    if (t < 0.5f) {
        float u = t * 2.0f;               // 0→1 over first half
        r = static_cast<uint8_t>(255 * u);
        g = 210;
        b = 0;
    } else {
        float u = (t - 0.5f) * 2.0f;      // 0→1 over second half
        r = 255;
        g = static_cast<uint8_t>(210 * (1.0f - u));
        b = 0;
    }
    return IM_COL32(r, g, b, 230);
}

// ---------------------------------------------------------------------------
void TransferMFD::setEpoch(const astro::EphemerisTime& et)
{
    const double now = et.getETValue();

    // Earth (399) → Mars barycenter (4), heliocentric (Sun = 10).
    m_params.departureBody = 399;
    m_params.arrivalBody   = 4;
    m_params.centralBody   = 10;
    m_params.muCentral     = 0.0;   // queried from SPICE on compute

    // Default window: departure over 2 years starting now.
    m_params.t0     = now;
    m_params.t1     = now + 2.0 * 365.25 * kDay;

    // TOF range: 100–400 days.
    m_params.tofMin = 100.0 * kDay;
    m_params.tofMax = 400.0 * kDay;

    m_params.nDep = 80;
    m_params.nTof = 60;
}

// ---------------------------------------------------------------------------
void TransferMFD::compute()
{
    if (m_computing) return;
    m_computing = true;
    m_hasData   = false;
    m_error.clear();

    try {
        m_data    = spacecraft::computePorkchop(m_params);
        m_hasData = true;
        m_selDep  = -1;
        m_selTof  = -1;
    } catch (const std::exception& e) {
        m_error = e.what();
    } catch (...) {
        m_error = "unknown exception";
    }

    m_computing = false;
}

// ---------------------------------------------------------------------------
const char* TransferMFD::leftLabel(int slot) const
{
    switch (slot) {
    case 0: return "DEP<";
    case 1: return "DEP>";
    case 2: return "TOF<";
    case 3: return "TOF>";
    case 4: return "COMP";
    default: return "";
    }
}

const char* TransferMFD::rightLabel(int slot) const
{
    switch (slot) {
    case 0: return "WIN<";
    case 1: return "WIN>";
    case 2: return "RNG<";
    case 3: return "RNG>";
    default: return "";
    }
}

void TransferMFD::onLeft(int slot)
{
    const double span = m_params.t1 - m_params.t0;
    const double shift = 30.0 * kDay;
    switch (slot) {
    case 0:  // DEP< — shift window back
        m_params.t0 -= shift;
        m_params.t1 -= shift;
        break;
    case 1:  // DEP> — shift window forward
        m_params.t0 += shift;
        m_params.t1 += shift;
        break;
    case 2:  // TOF< — shift TOF range down
        m_params.tofMin -= shift;
        m_params.tofMax -= shift;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    case 3:  // TOF> — shift TOF range up
        m_params.tofMin += shift;
        m_params.tofMax += shift;
        break;
    case 4:  // COMP
        compute();
        break;
    default: break;
    }
    (void)span;
}

void TransferMFD::onRight(int slot)
{
    const double depMid = (m_params.t0 + m_params.t1) * 0.5;
    const double tofMid = (m_params.tofMin + m_params.tofMax) * 0.5;
    double depHalf = (m_params.t1 - m_params.t0) * 0.5;
    double tofHalf = (m_params.tofMax - m_params.tofMin) * 0.5;

    switch (slot) {
    case 0:  // WIN< — halve departure span
        depHalf = std::max(depHalf * 0.5, 30.0 * kDay);
        m_params.t0 = depMid - depHalf;
        m_params.t1 = depMid + depHalf;
        break;
    case 1:  // WIN> — double departure span
        depHalf *= 2.0;
        m_params.t0 = depMid - depHalf;
        m_params.t1 = depMid + depHalf;
        break;
    case 2:  // RNG< — halve TOF span
        tofHalf = std::max(tofHalf * 0.5, 15.0 * kDay);
        m_params.tofMin = tofMid - tofHalf;
        m_params.tofMax = tofMid + tofHalf;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    case 3:  // RNG> — double TOF span
        tofHalf *= 2.0;
        m_params.tofMin = tofMid - tofHalf;
        m_params.tofMax = tofMid + tofHalf;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
void TransferMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen    = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim      = IM_COL32(  0, 140,  50, 140);
    const ImU32 kWhite    = IM_COL32(255, 255, 255, 220);
    const ImU32 kYellow   = IM_COL32(255, 220,   0, 220);
    const ImU32 kNoData   = IM_COL32( 40,  40,  40, 200);

    const float pad    = 4.0f;
    const float labelH = 11.0f;   // line height for axis labels
    const float infoH  = labelH * 3.0f + pad;   // info panel height at bottom

    // Grid area.
    const float gx0 = origin.x + pad;
    const float gy0 = origin.y + pad;
    const float gx1 = origin.x + size.x - pad;
    const float gy1 = origin.y + size.y - infoH - pad;
    const float gw  = gx1 - gx0;
    const float gh  = gy1 - gy0;

    // ---- "COMPUTING" overlay ----
    if (m_computing) {
        const char* msg = "COMPUTING...";
        ImVec2 tsz = ImGui::CalcTextSize(msg);
        dl->AddText({ gx0 + (gw - tsz.x) * 0.5f,
                      gy0 + (gh - tsz.y) * 0.5f }, kGreen, msg);
        return;
    }

    // ---- No data yet ----
    if (!m_hasData) {
        const char* msg1 = "XFER: Earth -> Mars";
        const char* msg2 = m_error.empty() ? "Press COMP to compute" : m_error.c_str();
        ImU32       col2 = m_error.empty() ? kDim : IM_COL32(255, 80, 80, 220);
        ImVec2 s1 = ImGui::CalcTextSize(msg1);
        ImVec2 s2 = ImGui::CalcTextSize(msg2);
        float cy = gy0 + (gh - labelH * 2.0f - pad) * 0.5f;
        dl->AddText({ gx0 + (gw - s1.x) * 0.5f,       cy             }, kGreen, msg1);
        dl->AddText({ gx0 + (gw - s2.x) * 0.5f, cy + labelH + pad }, col2,   msg2);
        return;
    }

    const int nDep = m_data.params.nDep;
    const int nTof = m_data.params.nTof;
    const float cellW = gw / static_cast<float>(nDep);
    const float cellH = gh / static_cast<float>(nTof);

    // ---- Draw grid cells ----
    // Remap ΔV: clamp at dvMin + 3*(dvMin) to saturate the scale sensibly.
    const float dvLo  = m_data.dvMin;
    const float dvHi  = std::min(m_data.dvMax, dvLo * 4.0f + 3.0f);  // km/s cap
    const float dvRng = (dvHi > dvLo) ? (dvHi - dvLo) : 1.0f;

    for (int iDep = 0; iDep < nDep; ++iDep) {
        for (int iTof = 0; iTof < nTof; ++iTof) {
            float dv  = m_data.get(m_data.dvTotal, iDep, iTof);
            float cx0 = gx0 + iDep       * cellW;
            float cy0 = gy0 + (nTof - 1 - iTof) * cellH;   // TOF increases upward

            ImU32 col;
            if (dv >= spacecraft::PorkchopData::kNoSolution * 0.5f) {
                col = kNoData;
            } else {
                float t = (dv - dvLo) / dvRng;
                col = dvColor(t);
            }
            dl->AddRectFilled({ cx0, cy0 }, { cx0 + cellW, cy0 + cellH }, col);
        }
    }

    // ---- Mouse hover ----
    const ImVec2 mp = ImGui::GetMousePos();
    const bool inGrid = mp.x >= gx0 && mp.x < gx1 && mp.y >= gy0 && mp.y < gy1;
    int hDep = -1, hTof = -1;
    if (inGrid) {
        hDep = static_cast<int>((mp.x - gx0) / cellW);
        hTof = nTof - 1 - static_cast<int>((mp.y - gy0) / cellH);
        hDep = std::clamp(hDep, 0, nDep - 1);
        hTof = std::clamp(hTof, 0, nTof - 1);

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            m_selDep = hDep;
            m_selTof = hTof;
        }
    }

    // ---- Selection crosshair ----
    if (m_selDep >= 0 && m_selTof >= 0) {
        float sx = gx0 + (m_selDep + 0.5f) * cellW;
        float sy = gy0 + (nTof - 1 - m_selTof + 0.5f) * cellH;
        const float cs = 5.0f;
        dl->AddLine({ sx - cs, sy }, { sx + cs, sy }, kWhite, 1.0f);
        dl->AddLine({ sx, sy - cs }, { sx, sy + cs }, kWhite, 1.0f);
    }

    // ---- Hover crosshair (dimmer) ----
    if (inGrid) {
        float sx = gx0 + (hDep + 0.5f) * cellW;
        float sy = gy0 + (nTof - 1 - hTof + 0.5f) * cellH;
        dl->AddLine({ gx0, sy }, { gx1, sy }, IM_COL32(255,255,255,50), 0.5f);
        dl->AddLine({ sx, gy0 }, { sx, gy1 }, IM_COL32(255,255,255,50), 0.5f);
    }

    // ---- Axis ticks ----
    // Departure: 5 ticks along bottom
    for (int i = 0; i <= 4; ++i) {
        float fx  = gx0 + gw * i / 4.0f;
        double et = m_data.params.t0 + (m_data.params.t1 - m_data.params.t0) * i / 4.0;
        std::string ts = astro::EphemerisTime(et).toISOUTCString(0);
        // Show only year-month for brevity
        const char* label = ts.size() >= 7 ? ts.c_str() : "?";
        char buf[8]; std::snprintf(buf, sizeof(buf), "%.7s", label);
        dl->AddLine({ fx, gy1 }, { fx, gy1 + 3.0f }, kDim, 1.0f);
        dl->AddText({ fx - 14.0f, gy1 + 2.0f }, kDim, buf);
    }
    // TOF: 3 ticks on left
    for (int i = 0; i <= 3; ++i) {
        float fy   = gy1 - gh * i / 3.0f;
        double tof = m_data.params.tofMin
                   + (m_data.params.tofMax - m_data.params.tofMin) * i / 3.0;
        int tofDays = static_cast<int>(tof / kDay + 0.5);
        char buf[8]; std::snprintf(buf, sizeof(buf), "%dd", tofDays);
        dl->AddLine({ gx0 - 3.0f, fy }, { gx0, fy }, kDim, 1.0f);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ gx0 - tsz.x - 2.0f, fy - tsz.y * 0.5f }, kDim, buf);
    }

    // ---- Info panel ----
    const float infoY = gy1 + labelH * 0.5f + pad;
    int di = (inGrid ? hDep : m_selDep);
    int ti = (inGrid ? hTof : m_selTof);

    if (di >= 0 && ti >= 0) {
        double depET  = m_data.depET(di);
        double tofSec = m_data.tofS(ti);
        float  dv1    = m_data.get(m_data.dv1,    di, ti);
        float  dv2    = m_data.get(m_data.dv2,    di, ti);
        float  dvTot  = m_data.get(m_data.dvTotal, di, ti);

        std::string depStr = astro::EphemerisTime(depET).toISOUTCString(0);
        int tofDays = static_cast<int>(tofSec / kDay + 0.5);

        char buf[64];
        std::snprintf(buf, sizeof(buf), "DEP  %.10s", depStr.c_str());
        dl->AddText({ origin.x + pad, infoY }, kGreen, buf);

        std::snprintf(buf, sizeof(buf), "TOF  %dd", tofDays);
        dl->AddText({ origin.x + pad, infoY + labelH }, kGreen, buf);

        if (dvTot < spacecraft::PorkchopData::kNoSolution * 0.5f) {
            std::snprintf(buf, sizeof(buf), "DV1 %.2f  DV2 %.2f  TOT %.2f km/s",
                          dv1, dv2, dvTot);
            dl->AddText({ origin.x + pad, infoY + 2.0f * labelH }, kYellow, buf);
        } else {
            dl->AddText({ origin.x + pad, infoY + 2.0f * labelH }, kDim, "no solution");
        }
    } else {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "dVmin %.2f km/s", m_data.dvMin);
        dl->AddText({ origin.x + pad, infoY }, kGreen, buf);
        dl->AddText({ origin.x + pad, infoY + labelH }, kDim, "hover/click to select");
    }
}
