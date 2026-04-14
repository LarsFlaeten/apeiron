#include "TransferMFD.h"
#include "OrbitDiagram.h"

#include <astro/SpiceCore.h>

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <iostream>

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
        const astro::SpiceException se = dynamic_cast<const astro::SpiceException&>(e);
        m_error = e.what();
        std::cout << "[TransferMFD] Compute error: " << e.what() << "\n";
        std::cout << "[TransferMFD]    Spice short message exp: " << se.getShortMessageExplanation() << "\n";
        std::cout << "[TransferMFD]    Spice long message: " << se.getLongMessage() << "\n";
        
    } catch (...) {
        m_error = "unknown exception";
    }

    m_computing = false;
}

// ---------------------------------------------------------------------------
const char* TransferMFD::leftLabel(int slot) const
{
    if (m_page == 2) {
        if (slot == 4) return "BACK";
        if (slot == 3) return "ALT";
        return "";
    }
    if (m_page == 1) {
        if (slot == 4) return "BACK";
        if (slot == 3) return "ALT";
        return "";
    }
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
    if (m_page == 2) return "";
    if (m_page == 1) {
        if (slot == 4) return "BURN";
        return "";
    }
    switch (slot) {
    case 0: return "WIN<";
    case 1: return "WIN>";
    case 2: return "RNG<";
    case 3: return "RNG>";
    case 4: return (m_selDep >= 0 && m_selTof >= 0 && m_hasData) ? "INFO" : "";
    default: return "";
    }
}

void TransferMFD::onLeft(int slot)
{
    if (m_page == 2) {
        if (slot == 4) m_page = 1;
        if (slot == 3) m_parkIdx = (m_parkIdx + 1)
                                   % static_cast<int>(std::size(kParkAlts));
        return;
    }
    if (m_page == 1) {
        if (slot == 4) m_page = 0;
        if (slot == 3) m_parkIdx = (m_parkIdx + 1)
                                   % static_cast<int>(std::size(kParkAlts));
        return;
    }
    const double shift = 30.0 * kDay;
    switch (slot) {
    case 0:
        m_params.t0 -= shift;
        m_params.t1 -= shift;
        break;
    case 1:
        m_params.t0 += shift;
        m_params.t1 += shift;
        break;
    case 2:
        m_params.tofMin -= shift;
        m_params.tofMax -= shift;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    case 3:
        m_params.tofMin += shift;
        m_params.tofMax += shift;
        break;
    case 4:
        compute();
        break;
    default: break;
    }
}

void TransferMFD::onRight(int slot)
{
    if (m_page == 2) return;
    if (m_page == 1) {
        if (slot == 4) m_page = 2;
        return;
    }
    const double depMid = (m_params.t0 + m_params.t1) * 0.5;
    const double tofMid = (m_params.tofMin + m_params.tofMax) * 0.5;
    double depHalf = (m_params.t1 - m_params.t0) * 0.5;
    double tofHalf = (m_params.tofMax - m_params.tofMin) * 0.5;

    switch (slot) {
    case 0:
        depHalf = std::max(depHalf * 0.5, 30.0 * kDay);
        m_params.t0 = depMid - depHalf;
        m_params.t1 = depMid + depHalf;
        break;
    case 1:
        depHalf *= 2.0;
        m_params.t0 = depMid - depHalf;
        m_params.t1 = depMid + depHalf;
        break;
    case 2:
        tofHalf = std::max(tofHalf * 0.5, 15.0 * kDay);
        m_params.tofMin = tofMid - tofHalf;
        m_params.tofMax = tofMid + tofHalf;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    case 3:
        tofHalf *= 2.0;
        m_params.tofMin = tofMid - tofHalf;
        m_params.tofMax = tofMid + tofHalf;
        if (m_params.tofMin < kDay) {
            m_params.tofMax -= m_params.tofMin - kDay;
            m_params.tofMin  = kDay;
        }
        break;
    case 4:  // INFO — open detail page for selected cell
        if (m_selDep >= 0 && m_selTof >= 0 && m_hasData) {
            resolveSelected();
            if (m_detail.valid) m_page = 1;
        }
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
void TransferMFD::resolveSelected()
{
    m_detail = {};
    if (!m_hasData || m_selDep < 0 || m_selTof < 0) return;

    const double depET  = m_data.depET(m_selDep);
    const double tofSec = m_data.tofS (m_selTof);
    const double arrET  = depET + tofSec;

    astro::PosState dep, arr;
    try {
        astro::Spice().getRelativeGeometricState(
            m_params.departureBody, m_params.centralBody,
            astro::EphemerisTime(depET), dep);
        astro::Spice().getRelativeGeometricState(
            m_params.arrivalBody, m_params.centralBody,
            astro::EphemerisTime(arrET), arr);
    } catch (...) { return; }

    double mu = m_params.muCentral;
    if (mu <= 0.0)
        astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", mu);

    glm::dvec3 vDep, vArr;
    if (!spacecraft::solveLambert(mu, dep.r, arr.r, tofSec, true, vDep, vArr))
        return;

    glm::dvec3 vInf = vDep - dep.v;
    m_detail.valid    = true;
    m_detail.depET    = depET;
    m_detail.arrET    = arrET;
    m_detail.tofSec   = tofSec;
    m_detail.dv1      = static_cast<float>(glm::length(vInf));
    m_detail.dv2      = static_cast<float>(glm::length(vArr - arr.v));
    m_detail.c3       = static_cast<float>(glm::dot(vInf, vInf));
    m_detail.depPos   = dep.r;
    m_detail.arrPos   = arr.r;
    m_detail.vDepBody = dep.v;
    m_detail.vArrBody = arr.v;
    m_detail.vDep     = vDep;
    m_detail.vArr     = vArr;
}

// ---------------------------------------------------------------------------
void TransferMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    if (m_page == 2) { renderDeparture(dl, origin, size); return; }
    if (m_page == 1) { renderDetail   (dl, origin, size); return; }
    renderPorkchop(dl, origin, size);
}

// ---------------------------------------------------------------------------
void TransferMFD::renderPorkchop(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen    = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim      = IM_COL32(  0, 140,  50, 140);
    const ImU32 kWhite    = IM_COL32(255, 255, 255, 220);
    const ImU32 kYellow   = IM_COL32(255, 220,   0, 220);
    const ImU32 kNoData   = IM_COL32( 40,  40,  40, 200);

    const float pad    = 4.0f;
    const float labelH = 11.0f;   // line height for axis labels
    // Info panel: 3 text lines + gap from grid bottom tick labels (labelH).
    const float infoH  = labelH * 3.0f + labelH + pad;

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

    // ---- Contour lines (marching squares) ----
    {
        // ΔV iso-levels to draw (km/s).
        static const float kLevels[] = { 6.f, 7.f, 8.f, 9.f, 10.f, 12.f, 15.f };
        static const int   kNLev     = static_cast<int>(std::size(kLevels));

        // Marching squares lookup: {eA,eB,eC,eD} — draw segment(eA→eB) and optional (eC→eD).
        // Cases 5 & 10 are saddles, handled separately.
        static const int8_t kMS[16][4] = {
            {-1,-1,-1,-1}, // 0  0000
            { 3, 0,-1,-1}, // 1  0001  v0 above
            { 0, 1,-1,-1}, // 2  0010  v1 above
            { 3, 1,-1,-1}, // 3  0011  v0+v1
            { 1, 2,-1,-1}, // 4  0100  v2
            { 0, 0, 0, 0}, // 5  0101  saddle
            { 0, 2,-1,-1}, // 6  0110  v1+v2
            { 3, 2,-1,-1}, // 7  0111  v0+v1+v2
            { 2, 3,-1,-1}, // 8  1000  v3
            { 2, 0,-1,-1}, // 9  1001  v0+v3
            { 0, 0, 0, 0}, //10  1010  saddle
            { 2, 1,-1,-1}, //11  1011  v0+v1+v3
            { 1, 3,-1,-1}, //12  1100  v2+v3
            { 1, 0,-1,-1}, //13  1101  v0+v2+v3
            { 0, 3,-1,-1}, //14  1110  v1+v2+v3
            {-1,-1,-1,-1}, //15  1111
        };

        // Interpolated screen position of threshold crossing on one edge of a square.
        // Square: data-space corners (i,j),(i+1,j),(i+1,j+1),(i,j+1) = v0,v1,v2,v3.
        // Edge: 0=bottom(v0-v1), 1=right(v1-v2), 2=top(v2-v3), 3=left(v3-v0).
        auto edgePt = [&](int i, int j, int edge, float lev,
                          float v0, float v1, float v2, float v3) -> ImVec2 {
            float bx = gx0 + (i + 0.5f) * cellW;
            float by = gy0 + (nTof - 0.5f - j) * cellH;
            float tx = bx + cellW;
            float ty = by - cellH;   // ty < by: higher j → higher on screen → smaller y
            switch (edge) {
            case 0: { float t=(lev-v0)/(v1-v0); return {bx+t*cellW, by         }; }
            case 1: { float t=(lev-v1)/(v2-v1); return {tx,          by-t*cellH }; }
            case 2: { float t=(lev-v2)/(v3-v2); return {tx-t*cellW,  ty         }; }
            case 3: { float t=(lev-v3)/(v0-v3); return {bx,          ty+t*cellH }; }
            default: return {bx, by};
            }
        };

        const float kNS = spacecraft::PorkchopData::kNoSolution * 0.5f;

        // Track one label point per level (where contour last crosses a right-side edge).
        struct LabelPt { float x, y; bool valid = false; };
        std::vector<LabelPt> lblPts(kNLev);

        for (int li = 0; li < kNLev; ++li) {
            const float lev = kLevels[li];
            if (lev < dvLo - 0.5f || lev > dvHi * 1.5f) continue;

            const ImU32 col = IM_COL32(255, 255, 255, 110);

            for (int i = 0; i < nDep - 1; ++i) {
                for (int j = 0; j < nTof - 1; ++j) {
                    float v0 = m_data.get(m_data.dvTotal, i,   j  );
                    float v1 = m_data.get(m_data.dvTotal, i+1, j  );
                    float v2 = m_data.get(m_data.dvTotal, i+1, j+1);
                    float v3 = m_data.get(m_data.dvTotal, i,   j+1);
                    if (v0>kNS || v1>kNS || v2>kNS || v3>kNS) continue;

                    int ci = ((v0>lev)?1:0)|((v1>lev)?2:0)|((v2>lev)?4:0)|((v3>lev)?8:0);
                    if (ci == 0 || ci == 15) continue;

                    int8_t e[4];
                    if (ci == 5 || ci == 10) {
                        // Disambiguate saddle by centre value.
                        bool ca = ((v0+v1+v2+v3)*0.25f > lev);
                        if (ci == 5) {
                            // v0,v2 above; v1,v3 below
                            if (!ca) { e[0]=3;e[1]=0;e[2]=1;e[3]=2; }
                            else     { e[0]=0;e[1]=1;e[2]=2;e[3]=3; }
                        } else {
                            // v1,v3 above; v0,v2 below
                            if (!ca) { e[0]=0;e[1]=1;e[2]=2;e[3]=3; }
                            else     { e[0]=3;e[1]=0;e[2]=1;e[3]=2; }
                        }
                    } else {
                        e[0]=kMS[ci][0]; e[1]=kMS[ci][1];
                        e[2]=kMS[ci][2]; e[3]=kMS[ci][3];
                    }

                    if (e[0]>=0 && e[1]>=0) {
                        ImVec2 pa = edgePt(i,j,e[0],lev,v0,v1,v2,v3);
                        ImVec2 pb = edgePt(i,j,e[1],lev,v0,v1,v2,v3);
                        dl->AddLine(pa, pb, col, 1.0f);
                        // Track label position: prefer middle column, accept any
                        if (!lblPts[li].valid || i > lblPts[li].x) {
                            lblPts[li] = {float(i), (pa.y+pb.y)*0.5f, true};
                        }
                    }
                    if (e[2]>=0 && e[3]>=0) {
                        ImVec2 pc = edgePt(i,j,e[2],lev,v0,v1,v2,v3);
                        ImVec2 pd = edgePt(i,j,e[3],lev,v0,v1,v2,v3);
                        dl->AddLine(pc, pd, col, 1.0f);
                    }
                }
            }
        }

        // Draw inline labels at the rightmost crossing of each level.
        for (int li = 0; li < kNLev; ++li) {
            if (!lblPts[li].valid) continue;
            char buf[6]; std::snprintf(buf, sizeof(buf), "%.0f", kLevels[li]);
            ImVec2 tsz = ImGui::CalcTextSize(buf);
            float lx = gx0 + (lblPts[li].x + 1.5f) * cellW - tsz.x * 0.5f;
            float ly = lblPts[li].y - tsz.y * 0.5f;
            // Small dark backing so text is readable over the colour map.
            dl->AddRectFilled({lx-1,ly-1}, {lx+tsz.x+1,ly+tsz.y+1},
                              IM_COL32(0,0,0,160));
            dl->AddText({lx, ly}, IM_COL32(255,255,200,220), buf);
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
    const float infoY = gy1 + labelH + pad;   // below tick labels
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


// ---------------------------------------------------------------------------
void TransferMFD::renderDetail(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan   = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange = IM_COL32(255, 140,   0, 220);

    const float pad   = 4.0f;
    const float lineH = 11.0f;
    const float cx    = origin.x + pad;
    float y           = origin.y + pad;

    if (!m_detail.valid) {
        dl->AddText({cx, y}, kDim, "No transfer selected");
        return;
    }

    // Mu — needed throughout.
    double mu = m_params.muCentral;
    if (mu <= 0.0) {
        try { astro::Spice().getPlanetaryConstants(m_params.centralBody, "GM", mu); }
        catch (...) {}
    }

    // ---- v∞ and burn geometry ----
    glm::dvec3 vInf    = m_detail.vDep - m_detail.vDepBody;
    double     vInfMag = glm::length(vInf);

    // Earth GM for TMI burn.
    double muEarth = 0.0;
    try { astro::Spice().getPlanetaryConstants(399, "GM", muEarth); }
    catch (...) { muEarth = 398600.4418; }

    const double rPark   = kParkAlts[m_parkIdx];
    const double altKm   = rPark - 6378.0;
    const double vCirc   = std::sqrt(muEarth / rPark);         // circular speed
    const double vPeri   = std::sqrt(m_detail.c3 + 2.0 * muEarth / rPark);  // hyperbolic periapsis speed
    const double dvTMI   = vPeri - vCirc;                      // TMI burn ΔV

    // Inclination of v∞ above ecliptic = angle between v∞ and ecliptic plane.
    // sin(inc) = vInf.z / |vInf|  (ecliptic frame: z is out-of-plane)
    double vInfInc = 0.0;
    if (vInfMag > 1e-6)
        vInfInc = std::asin(std::clamp(vInf.z / vInfMag, -1.0, 1.0)) * 180.0 / M_PI;

    // ---- Text rows ----
    auto row = [&](ImU32 col, const char* fmt, ...) {
        char buf[80];
        va_list ap; va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        dl->AddText({cx, y}, col, buf);
        y += lineH;
    };

    std::string nowStr = astro::EphemerisTime(m_currentET).toISOUTCString(0);
    std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
    std::string arrStr = astro::EphemerisTime(m_detail.arrET).toISOUTCString(0);
    int tofDays    = static_cast<int>(m_detail.tofSec / kDay + 0.5);
    int daysToLaunch = static_cast<int>((m_detail.depET - m_currentET) / kDay + 0.5);

    row(kDim,    "NOW  %.10s", nowStr.c_str());
    row(kGreen,  "DEP  %.10s  (T%+d d)", depStr.c_str(), daysToLaunch);
    row(kGreen,  "ARR  %.10s", arrStr.c_str());
    row(kGreen,  "TOF  %d days", tofDays);
    y += pad;
    row(kYellow, "DV1  %.3f km/s", m_detail.dv1);
    row(kYellow, "DV2  %.3f km/s", m_detail.dv2);
    row(kYellow, "TOT  %.3f km/s", m_detail.dv1 + m_detail.dv2);
    y += pad;
    row(kCyan,   "C3   %.2f km2/s2", m_detail.c3);

    if (vInfMag > 1e-6) {
        glm::dvec3 vn  = vInf / vInfMag;
        double lon = std::atan2(vn.y, vn.x) * 180.0 / M_PI;
        if (lon < 0.0) lon += 360.0;
        double lat = std::asin(std::clamp(vn.z, -1.0, 1.0)) * 180.0 / M_PI;
        row(kCyan, "VINF LON %.0f  LAT %.1f", lon, lat);
    }
    y += pad;
    row(kDim,    "TMI  alt %.0f km", altKm);
    row(kGreen,  "  Vcirc  %.3f km/s", vCirc);
    row(kGreen,  "  Vperi  %.3f km/s", vPeri);
    row(kYellow, "  dV-TMI %.3f km/s", dvTMI);
    if (std::abs(vInfInc) > 0.5)
        row(kOrange, "  inc    %.1f deg above ecl", vInfInc);

    // ---- Orbit diagram via OrbitDiagram ----
    const float diagH = size.y - (y - origin.y) - pad;
    if (diagH < 20.0f || mu <= 0.0) return;

    // Build transfer arc as a 3D polyline.
    OrbitDiagram::Arc transferArc;
    transferArc.colour    = kYellow;
    transferArc.thickness = 1.5f;
    {
        glm::dvec3 h    = glm::cross(m_detail.depPos, m_detail.vDep);
        double     hMag = glm::length(h);
        if (hMag > 1e-6) {
            glm::dvec3 hn      = h / hMag;
            double     p_      = hMag * hMag / mu;
            glm::dvec3 ev      = glm::cross(m_detail.vDep, h) / mu
                               - glm::normalize(m_detail.depPos);
            double     ecc     = glm::length(ev);
            glm::dvec3 periDir = (ecc > 1e-9)
                               ? glm::normalize(ev) : glm::normalize(m_detail.depPos);
            glm::dvec3 qDir    = glm::cross(hn, periDir);

            auto nu_of = [&](const glm::dvec3& r) -> double {
                double cosnu = glm::dot(periDir, glm::normalize(r));
                cosnu = std::clamp(cosnu, -1.0, 1.0);
                double nu = std::acos(cosnu);
                if (glm::dot(glm::cross(periDir, r), hn) < 0.0) nu = -nu;
                return nu;
            };
            double nu1 = nu_of(m_detail.depPos);
            double nu2 = nu_of(m_detail.arrPos);
            if (nu2 < nu1) nu2 += 2.0 * M_PI;

            for (int k = 0; k <= 80; ++k) {
                double nu = nu1 + (nu2 - nu1) * k / 80.0;
                double r_ = p_ / (1.0 + ecc * std::cos(nu));
                transferArc.pts.push_back(
                    r_ * (std::cos(nu) * periDir + std::sin(nu) * qDir));
            }
        }
    }

    OrbitDiagram diag;
    diag.addOrbit(m_detail.depPos, m_detail.vDepBody, mu,
                  IM_COL32(60, 140, 255, 90), "", true, false);   // Earth orbit
    diag.addOrbit(m_detail.arrPos, m_detail.vArrBody, mu,
                  IM_COL32(200, 80, 50, 90),  "", true, false);   // Mars orbit
    diag.addArc(transferArc);
    diag.addMarker(m_detail.depPos, IM_COL32( 60,140,255,255), "E");
    diag.addMarker(m_detail.arrPos, IM_COL32(200, 80, 50,255), "M");
    if (vInfMag > 1e-6)
        diag.addArrow(m_detail.depPos, vInf / vInfMag, 14.0f, kOrange);

    diag.render(dl, { origin.x, y + pad }, { size.x, diagH }, &m_detailViewRot);
}

// ---------------------------------------------------------------------------
// Page 2: geocentric departure burn planning.
//
// Geometry:
//   - V∞  = vDep_helio − vEarth_helio  (direction same in any inertial frame)
//   - Target orbit plane: closest plane containing V∞
//     hTarget = normalize( hCurrent − (hCurrent·V∞̂)·V∞̂ )
//   - AN of current orbit w.r.t. target plane:  cross(hTarget, hCurrent)
//   - Optimal burn TA ν*: maximise v(ν)·V∞_proj
//     ν* = atan2(−(V∞_proj·periDir), V∞_proj·qDir)
//   - Plane error = asin(|V∞̂·hCurrent|)
//
// Layout: diagram fills full area; all text overlaid with dark backing boxes.
// ---------------------------------------------------------------------------
void TransferMFD::renderDeparture(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 140);
    const ImU32 kYellow = IM_COL32(255, 220,   0, 230);
    const ImU32 kCyan   = IM_COL32(  0, 200, 220, 220);
    const ImU32 kOrange = IM_COL32(255, 140,   0, 220);
    const ImU32 kRed    = IM_COL32(255,  60,  60, 220);
    const ImU32 kBacking = IM_COL32(0, 0, 0, 175);

    // -----------------------------------------------------------------------
    // Guard conditions — need valid transfer and ship state
    // -----------------------------------------------------------------------
    if (!m_detail.valid) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim, "No transfer selected");
        return;
    }
    const double rShip   = glm::length(m_shipR);
    const double vShipMag = glm::length(m_shipV);
    if (rShip < 100.0 || vShipMag < 0.01) {
        dl->AddText({ origin.x + 4.0f, origin.y + 4.0f }, kDim, "No ship state");
        return;
    }

    // -----------------------------------------------------------------------
    // Orbital geometry — current parking orbit
    // -----------------------------------------------------------------------
    const double mu   = m_muEarth;
    const double kRE  = 6378.0;

    glm::dvec3 h    = glm::cross(m_shipR, m_shipV);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) { dl->AddText({origin.x+4,origin.y+4}, kDim, "Degenerate orbit"); return; }

    glm::dvec3 hHat    = h / hMag;
    double     p_orb   = hMag * hMag / mu;
    glm::dvec3 ev      = glm::cross(m_shipV, h) / mu - glm::normalize(m_shipR);
    double     ecc     = glm::length(ev);
    glm::dvec3 periDir = (ecc > 1e-6) ? glm::normalize(ev) : glm::normalize(m_shipR);
    glm::dvec3 qDir    = glm::cross(hHat, periDir);
    double     sma     = (ecc < 1.0) ? p_orb / (1.0 - ecc * ecc) : p_orb;
    double     altKm   = rShip - kRE;

    // Ecliptic orbital elements (ECLIPJ2000: z = ecliptic north, x = vernal equinox)
    double inc_cur  = std::acos(std::clamp(hHat.z, -1.0, 1.0)) * 180.0 / M_PI;
    double raan_cur = std::atan2(hHat.x, -hHat.y) * 180.0 / M_PI;
    if (raan_cur < 0.0) raan_cur += 360.0;

    // Argument of periapsis: angle from node line to periDir in orbit plane
    double nodeLen   = std::sqrt(hHat.x*hHat.x + hHat.y*hHat.y);
    double argpe_cur = 0.0;
    if (nodeLen > 1e-9 && ecc > 1e-6) {
        glm::dvec3 nodeDir(-hHat.y / nodeLen, hHat.x / nodeLen, 0.0);
        argpe_cur = std::atan2(
            glm::dot(periDir, glm::cross(hHat, nodeDir)),
            glm::dot(periDir, nodeDir)) * 180.0 / M_PI;
        if (argpe_cur < 0.0) argpe_cur += 360.0;
    }

    // Current true anomaly
    glm::dvec3 rHat   = glm::normalize(m_shipR);
    double     ta_now = std::atan2(glm::dot(rHat, qDir), glm::dot(rHat, periDir));
    double     taDeg_now = ta_now * 180.0 / M_PI;
    if (taDeg_now < 0.0) taDeg_now += 360.0;

    // -----------------------------------------------------------------------
    // V∞
    // -----------------------------------------------------------------------
    glm::dvec3 vInf    = m_detail.vDep - m_detail.vDepBody;
    double     vInfMag = glm::length(vInf);
    if (vInfMag < 1e-6) { dl->AddText({origin.x+4,origin.y+4}, kDim, "Zero V-inf"); return; }
    glm::dvec3 vInfHat = vInf / vInfMag;

    // -----------------------------------------------------------------------
    // Target orbit plane: closest to current, constrained to contain V∞
    // hTarget = normalize( hHat − (hHat·V∞̂)·V∞̂ )
    // -----------------------------------------------------------------------
    glm::dvec3 hProj   = hHat - glm::dot(hHat, vInfHat) * vInfHat;
    double     hProjL  = glm::length(hProj);
    glm::dvec3 hTarget = (hProjL > 1e-9) ? (hProj / hProjL)
                       : glm::normalize(glm::cross(vInfHat, glm::dvec3(0,0,1)));

    double inc_tgt  = std::acos(std::clamp(hTarget.z, -1.0, 1.0)) * 180.0 / M_PI;
    double raan_tgt = std::atan2(hTarget.x, -hTarget.y) * 180.0 / M_PI;
    if (raan_tgt < 0.0) raan_tgt += 360.0;
    double dInc = inc_tgt - inc_cur;

    // -----------------------------------------------------------------------
    // Plane error
    // -----------------------------------------------------------------------
    double planeErr = std::asin(std::clamp(std::abs(glm::dot(vInfHat, hHat)), 0.0, 1.0))
                      * 180.0 / M_PI;

    // -----------------------------------------------------------------------
    // AN / DN of current orbit w.r.t. target plane
    // AN direction = cross(hTarget, hCurrent)
    // -----------------------------------------------------------------------
    glm::dvec3 anVec = glm::cross(hTarget, hHat);
    double     anLen = glm::length(anVec);
    bool       hasNodes = (anLen > 1e-9);

    double ta_AN = 0.0, ta_DN = 0.0;
    double taDeg_AN = 0.0, taDeg_DN = 0.0;
    double tToAN = 0.0, tToDN = 0.0;
    glm::dvec3 anPos(0.0), dnPos(0.0);

    if (hasNodes) {
        glm::dvec3 anDir = anVec / anLen;
        glm::dvec3 dnDir = -anDir;

        ta_AN = std::atan2(glm::dot(anDir, qDir), glm::dot(anDir, periDir));
        ta_DN = ta_AN + M_PI;
        // Normalise DN to (-π, π)
        if (ta_DN >  M_PI) ta_DN -= 2.0 * M_PI;

        taDeg_AN = ta_AN * 180.0 / M_PI; if (taDeg_AN < 0.0) taDeg_AN += 360.0;
        taDeg_DN = ta_DN * 180.0 / M_PI; if (taDeg_DN < 0.0) taDeg_DN += 360.0;

        double rAN = p_orb / (1.0 + ecc * std::cos(ta_AN));
        double rDN = p_orb / (1.0 + ecc * std::cos(ta_DN));
        anPos = rAN * (std::cos(ta_AN) * periDir + std::sin(ta_AN) * qDir);
        dnPos = rDN * (std::cos(ta_DN) * periDir + std::sin(ta_DN) * qDir);

        // Time to AN/DN via Kepler (elliptic only)
        if (ecc < 1.0 && sma > 0.0) {
            auto ta2M = [&](double ta) {
                double E = 2.0 * std::atan2(
                    std::sqrt(1.0 - ecc) * std::sin(ta * 0.5),
                    std::sqrt(1.0 + ecc) * std::cos(ta * 0.5));
                return E - ecc * std::sin(E);
            };
            double n_mot = std::sqrt(mu / (sma * sma * sma));
            double M_now = ta2M(ta_now);
            double dM_AN = ta2M(ta_AN) - M_now; if (dM_AN <= 0.0) dM_AN += 2.0 * M_PI;
            double dM_DN = ta2M(ta_DN) - M_now; if (dM_DN <= 0.0) dM_DN += 2.0 * M_PI;
            tToAN = dM_AN / n_mot;
            tToDN = dM_DN / n_mot;
        }
    }

    // -----------------------------------------------------------------------
    // Optimal burn TA
    // -----------------------------------------------------------------------
    glm::dvec3 vInfProj    = vInf - glm::dot(vInf, hHat) * hHat;
    double     vInfProjMag = glm::length(vInfProj);
    double burnTA = 0.0, burnTADeg = 0.0;
    if (vInfProjMag > 1e-9) {
        double vp = glm::dot(vInfProj, periDir);
        double vq = glm::dot(vInfProj, qDir);
        burnTA = std::atan2(-vp, vq);
        double chk = -std::sin(burnTA)*vp + (ecc + std::cos(burnTA))*vq;
        if (chk < 0.0) burnTA += M_PI;
        if (burnTA >  M_PI) burnTA -= 2.0 * M_PI;
        if (burnTA < -M_PI) burnTA += 2.0 * M_PI;
        burnTADeg = burnTA * 180.0 / M_PI;
        if (burnTADeg < 0.0) burnTADeg += 360.0;
    }
    double rBurn   = p_orb / (1.0 + ecc * std::cos(burnTA));
    double altBurn = rBurn - kRE;
    glm::dvec3 burnPos = rBurn * (std::cos(burnTA) * periDir + std::sin(burnTA) * qDir);

    // TMI ΔV
    const double rPark = kParkAlts[m_parkIdx];
    double vCirc = std::sqrt(mu / rPark);
    double vPeri = std::sqrt(m_detail.c3 + 2.0 * mu / rPark);
    double dvTMI = vPeri - vCirc;

    // -----------------------------------------------------------------------
    // Burn timing
    // -----------------------------------------------------------------------
    // Time from current TA to burn TA via Kepler's equation (elliptic only).
    double timeToBurnPoint = 0.0;
    double orbPeriod       = 0.0;
    if (ecc < 1.0 && sma > 0.0) {
        orbPeriod = 2.0 * M_PI * std::sqrt(sma * sma * sma / mu);
        auto meanAnom = [&](double ta) -> double {
            double tanH  = std::tan(ta * 0.5) * std::sqrt((1.0 - ecc) / (1.0 + ecc));
            double E     = 2.0 * std::atan(tanH);
            return E - ecc * std::sin(E);
        };
        double M_now  = meanAnom(ta_now);
        double M_burn = meanAnom(burnTA);
        double dM = M_burn - M_now;
        if (dM < 0.0) dM += 2.0 * M_PI;   // wrap to next occurrence
        timeToBurnPoint = dM / (2.0 * M_PI) * orbPeriod;
    }

    // Burn duration and remaining ΔV.
    // accel in m/s², dvTMI in km/s → multiply by 1000 for m/s.
    const double accelMs2    = (m_shipMass > 1.0) ? m_mainThrustN / m_shipMass : 0.97;
    const double burnDuration = dvTMI * 1000.0 / accelMs2;   // seconds for full burn

    // Live remaining ΔV: vPeri - current speed (km/s). Approaches zero as burn completes.
    const double currentSpeed = glm::length(m_shipV);         // km/s
    const double dvRemaining  = std::max(0.0, vPeri - currentSpeed);
    const double burnRemaining = dvRemaining * 1000.0 / accelMs2; // seconds left

    // Time to start burn = time to burn point - half burn duration (periapsis-centered).
    const double timeToStartBurn = timeToBurnPoint - burnDuration * 0.5;

    // -----------------------------------------------------------------------
    // HH:MM:SS formatter
    // -----------------------------------------------------------------------
    auto fmtHMS = [](double sec, char* buf, int sz) {
        if (sec < 0.0 || !std::isfinite(sec)) { std::snprintf(buf, sz, "--:--:--"); return; }
        int s = static_cast<int>(sec + 0.5);
        int hh = s / 3600; s %= 3600;
        int mm = s / 60;   s %= 60;
        std::snprintf(buf, sz, "%d:%02d:%02d", hh, mm, s);
    };

    // -----------------------------------------------------------------------
    // Orbit diagram — full area
    // -----------------------------------------------------------------------
    {
        OrbitDiagram diag;

        // Parking orbit with ship dot
        OrbitDiagram::Orbit o;
        o.r = m_shipR; o.v = m_shipV; o.mu = mu;
        o.colour = kGreen; o.showApses = (ecc > 0.005); o.showCurrent = true;
        diag.addOrbit(o);

        // Target orbit plane visualised as a ghosted ring at same SMA
        // (no real velocity, just a fake circular orbit in the target plane to show the plane)
        {
            // pick a periDir perpendicular to hTarget in the target plane
            glm::dvec3 tPeri = glm::normalize(glm::cross(hTarget, hHat));
            if (glm::length(tPeri) < 0.5)
                tPeri = glm::normalize(glm::cross(hTarget, glm::dvec3(1,0,0)));
            OrbitDiagram::Orbit ot;
            double rcirc   = std::sqrt(mu * sma); // circular speed at current SMA
            ot.r     = sma * tPeri;
            ot.v     = std::sqrt(mu / sma) * glm::cross(hTarget, tPeri);
            ot.mu    = mu;
            ot.colour = IM_COL32(60, 180, 255, 60);  // dim blue ghost
            ot.showApses = false; ot.showCurrent = false;
            diag.addOrbit(ot);
        }

        // AN / DN markers
        if (hasNodes) {
            diag.addMarker(anPos, IM_COL32(100, 255, 100, 230), "AN");
            diag.addMarker(dnPos, IM_COL32(255, 100, 100, 230), "DN");
        }

        // Burn point
        diag.addMarker(burnPos, kYellow, "B");

        // V∞ direction arrow from Earth centre
        if (vInfProjMag > 1e-9)
            diag.addArrow({0.0, 0.0, 0.0}, vInfHat, 22.0f, kOrange);

        // Earth
        OrbitDiagram::CentralBody earth;
        earth.radiusKm   = kRE;
        earth.rimColour  = IM_COL32(60, 140, 255, 200);
        earth.axisColour = IM_COL32(60, 140, 255, 100);
        earth.drawAxes   = false;
        diag.setCentralBody(earth);

        diag.render(dl, origin, size, &m_depViewRot);
    }

    // -----------------------------------------------------------------------
    // Text overlay — build lines, then draw with one backing rect
    // -----------------------------------------------------------------------
    const float pad   = 3.0f;
    const float lineH = 11.0f;

    struct Line { ImU32 col; char txt[72]; };
    std::vector<Line> lines;
    auto add = [&](ImU32 col, const char* fmt, ...) {
        Line l; l.col = col;
        va_list ap; va_start(ap, fmt);
        std::vsnprintf(l.txt, sizeof(l.txt), fmt, ap);
        va_end(ap);
        lines.push_back(l);
    };
    auto sep = [&]() { add(0, ""); };  // blank spacer

    // Header — dates and V∞
    {
        std::string nowStr = astro::EphemerisTime(m_currentET).toISOUTCString(0);
        std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
        int daysToLaunch   = static_cast<int>((m_detail.depET - m_currentET) / kDay + 0.5);
        add(kDim,   "NOW  %.10s", nowStr.c_str());
        add(kGreen, "DEP  %.10s  (T%+d d)", depStr.c_str(), daysToLaunch);
    }
    add(kCyan,  "V-inf %.3f km/s  C3 %.1f km2/s2", vInfMag, m_detail.c3);
    sep();

    // Current orbit
    add(kCyan,   "CURRENT ORBIT");
    add(kGreen,  " i %.1f  Ω %.1f  ω %.1f  e %.4f",
                 inc_cur, raan_cur, argpe_cur, ecc);
    add(kGreen,  " TA %.1f  Alt %.0f km  SMA %.0f km",
                 taDeg_now, altKm, sma);
    sep();

    // -----------------------------------------------------------------------
    // Plane-change costs
    //
    // Direct (at current LEO):  dV_pc = 2·v_c·sin(Δi/2)
    //
    // Bi-elliptic (raise to r_a, plane change at apoapsis, TMI at periapsis):
    //   v_apo(r_a)  = sqrt(2·μ·r_c / (r_a·(r_c+r_a)))
    //   dV_pc(r_a)  = 2·v_apo·sin(Δi/2)
    //   dV_total    = (v_hyp − v_c) + dV_pc(r_a)   ← same base TMI cost
    //   t_to_apo    = π·sqrt((r_c+r_a)³ / (8·μ))   (half-period of transfer ellipse)
    //
    // Orion main engine accel ≈ 0.97 m/s² (25700 N / 26500 kg)
    // -----------------------------------------------------------------------
    const double sinHalfDi = std::sin(planeErr * M_PI / 180.0 * 0.5);
    const double vCircPark = std::sqrt(mu / rPark);   // circular at reference alt
    const double vHyp      = std::sqrt(m_detail.c3 + 2.0 * mu / rPark);
    const double dvTMIbase = vHyp - vCircPark;        // base TMI ΔV (independent of plane strat)

    // Direct plane change at current LEO speed
    double vLEO   = std::sqrt(mu / rShip);
    double dvPCdirect = 2.0 * vLEO * sinHalfDi;

    // Orion thrust/mass
    const double thrustN  = 25700.0;
    const double massKg   = 26500.0;
    const double accel    = thrustN / massKg;   // m/s²

    // Bi-elliptic candidates: GEO, HEO, Lunar distance
    struct BiElliptic { const char* label; double r_a; };
    static const BiElliptic kCands[] = {
        { "GEO  42k km", 42164.0  },
        { "HEO 100k km", 100000.0 },
        { "Lun 384k km", 384400.0 },
    };

    // Target plane
    ImU32 tgtCol = (planeErr < 0.5) ? kGreen : (planeErr < 5.0) ? kOrange : kRed;
    add(kCyan,   "TARGET PLANE  (contains V-inf)");
    add(kGreen,  " i %.1f  Ω %.1f", inc_tgt, raan_tgt);
    add(tgtCol,  " Di %.1f deg  Perr %.1f deg%s",
                 dInc, planeErr, planeErr > 5.0 ? " NEED CHG" : "");
    if (planeErr > 0.05) {
        double burnTimeDirect = dvPCdirect * 1000.0 / accel;
        char bufD[12]; fmtHMS(burnTimeDirect, bufD, sizeof(bufD));
        add(tgtCol, " direct dV %.2f km/s  burn %s", dvPCdirect, bufD);
    }
    sep();

    // Bi-elliptic section (only show if plane change is non-trivial)
    if (planeErr > 0.5) {
        add(kCyan, "BI-ELLIPTIC  (raise apo, chg plane there, TMI at peri)");
        add(kDim,  " base TMI dV unchanged: %.3f km/s", dvTMIbase);
        for (const auto& c : kCands) {
            double r_a   = c.r_a;
            double v_apo = std::sqrt(2.0 * mu * rShip / (r_a * (rShip + r_a)));
            double dvPCbi = 2.0 * v_apo * sinHalfDi;
            double saving = dvPCdirect - dvPCbi;
            // Time to reach apoapsis = half transfer orbit period
            double a_tr  = (rShip + r_a) * 0.5;
            double tApo  = M_PI * std::sqrt(a_tr * a_tr * a_tr / mu);  // seconds
            char bufT[12]; fmtHMS(tApo, bufT, sizeof(bufT));
            ImU32 col = (saving > 0.5) ? kGreen : (saving > 0.1) ? kYellow : kDim;
            add(col, " %-12s  dV-pc %.2f  save %.2f  T+ %s",
                c.label, dvPCbi, saving, bufT);
        }
    }
    sep();

    // Nodes
    add(kCyan,   "NODES  (cur -> tgt)");
    if (hasNodes) {
        char bufAN[12], bufDN[12];
        fmtHMS(tToAN, bufAN, sizeof(bufAN));
        fmtHMS(tToDN, bufDN, sizeof(bufDN));
        add(IM_COL32(100,255,100,230), " AN  TA %.1f  T+ %s", taDeg_AN, bufAN);
        add(IM_COL32(255,100,100,230), " DN  TA %.1f  T+ %s", taDeg_DN, bufDN);
    } else {
        add(kDim, " (coplanar)");
    }
    sep();

    // Burn + TMI
    add(kCyan,   "BURN  TA %.1f  Alt %.0f km", burnTADeg, altBurn);
    add(kDim,    "TMI  alt %.0f km  [ALT]", rPark - kRE);
    add(kYellow, " dV-TMI %.3f km/s  (Vp %.3f  Vc %.3f)", dvTMI, vPeri, vCirc);
    {
        char bufBurn[12], bufStart[12], bufToBurn[12];
        fmtHMS(burnDuration,    bufBurn,   sizeof(bufBurn));
        fmtHMS(timeToBurnPoint, bufToBurn, sizeof(bufToBurn));
        fmtHMS(std::max(0.0, timeToStartBurn), bufStart, sizeof(bufStart));
        add(kDim,    " accel %.3f m/s²  burn dur %s", accelMs2, bufBurn);
        add(kGreen,  " T-to-burn-point %s", bufToBurn);
        add(kYellow, " T-to-ignition   %s  (T-0.5burn)", bufStart);
    }
    sep();
    // Live ΔV remaining — updates in real time as the engine burns.
    {
        char bufRem[12];
        fmtHMS(burnRemaining, bufRem, sizeof(bufRem));
        ImU32 dvCol = (dvRemaining < 0.001) ? kGreen :
                      (dvRemaining < dvTMI * 0.1) ? kYellow : kOrange;
        add(dvCol, "dV-REMAINING %.3f km/s  (%s)", dvRemaining, bufRem);
        if (dvRemaining < 0.001)
            add(kGreen, " BURN COMPLETE");
    }

    // Measure max text width
    float maxW = 0.0f;
    for (auto& l : lines)
        if (l.col) maxW = std::max(maxW, ImGui::CalcTextSize(l.txt).x);

    float bx0 = origin.x + pad;
    float by0 = origin.y + pad;
    float bx1 = bx0 + maxW + pad * 2.0f;
    float by1 = by0 + static_cast<float>(lines.size()) * lineH + pad;

    dl->AddRectFilled({bx0, by0}, {bx1, by1}, kBacking, 3.0f);

    float tx = bx0 + pad, ty = by0 + pad * 0.5f;
    for (auto& l : lines) {
        if (l.col) dl->AddText({tx, ty}, l.col, l.txt);
        ty += lineH;
    }
}
