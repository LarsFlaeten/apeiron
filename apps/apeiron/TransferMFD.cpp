#include "TransferMFD.h"

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
    if (m_page == 1) return "";
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
    if (m_page == 1) return;
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
    if (m_page == 1) { renderDetail(dl, origin, size); return; }
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
// Draw one complete orbital ellipse from a state vector (r, v) around mu.
// Projected onto the ecliptic x-y plane (z ignored for display).
static void drawOrbit(ImDrawList* dl, const glm::dvec3& r, const glm::dvec3& v,
                      double mu, ImVec2 centre, double scale, ImU32 col)
{
    glm::dvec3 h    = glm::cross(r, v);
    double     hMag = glm::length(h);
    if (hMag < 1e-6) return;
    glm::dvec3 hn  = h / hMag;
    double     p_  = hMag * hMag / mu;
    glm::dvec3 ev  = glm::cross(v, h) / mu - glm::normalize(r);
    double     ecc = glm::length(ev);
    if (ecc >= 1.0) return;   // skip hyperbolic/parabolic

    glm::dvec3 periDir = (ecc > 1e-9) ? glm::normalize(ev) : glm::normalize(r);
    glm::dvec3 qDir    = glm::cross(hn, periDir);

    ImVec2 prev{}; bool first = true;
    for (int k = 0; k <= 120; ++k) {
        double nu  = 2.0 * M_PI * k / 120.0;
        double r_  = p_ / (1.0 + ecc * std::cos(nu));
        glm::dvec3 pos = r_ * (std::cos(nu) * periDir + std::sin(nu) * qDir);
        ImVec2 sp = { centre.x + static_cast<float>(pos.x * scale),
                      centre.y - static_cast<float>(pos.y * scale) };
        if (!first) dl->AddLine(prev, sp, col, 1.0f);
        prev = sp; first = false;
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

    std::string depStr = astro::EphemerisTime(m_detail.depET).toISOUTCString(0);
    std::string arrStr = astro::EphemerisTime(m_detail.arrET).toISOUTCString(0);
    int tofDays = static_cast<int>(m_detail.tofSec / kDay + 0.5);

    row(kGreen,  "DEP  %.10s", depStr.c_str());
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

    // ---- Orbit diagram ----
    const float diagSize = std::min(size.x, size.y - (y - origin.y)) - 2.0f * pad;
    if (diagSize < 20.0f || mu <= 0.0) return;

    const float dCx = origin.x + size.x * 0.5f;
    const float dCy = y + pad + diagSize * 0.5f;
    const float dR  = diagSize * 0.5f - 4.0f;

    double depR  = glm::length(m_detail.depPos);
    double arrR  = glm::length(m_detail.arrPos);
    double scale = dR / (std::max(depR, arrR) * 1.05);

    auto toScreen = [&](const glm::dvec3& p) -> ImVec2 {
        return { dCx + static_cast<float>(p.x * scale),
                 dCy - static_cast<float>(p.y * scale) };
    };

    // Real orbital ellipses from SPICE state vectors.
    drawOrbit(dl, m_detail.depPos, m_detail.vDepBody, mu,
              {dCx, dCy}, scale, IM_COL32(60, 140, 255, 80));   // Earth — blue
    drawOrbit(dl, m_detail.arrPos, m_detail.vArrBody, mu,
              {dCx, dCy}, scale, IM_COL32(200, 80, 50, 80));    // Mars  — red

    // Sun.
    dl->AddCircleFilled({dCx, dCy}, 4.0f, IM_COL32(255, 220, 60, 240));

    // Transfer arc.
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
                               ? glm::normalize(ev)
                               : glm::normalize(m_detail.depPos);
            glm::dvec3 qDir    = glm::cross(hn, periDir);

            auto trueAnom = [&](const glm::dvec3& r) -> double {
                double cosnu = glm::dot(periDir, glm::normalize(r));
                cosnu = std::clamp(cosnu, -1.0, 1.0);
                double nu = std::acos(cosnu);
                if (glm::dot(glm::cross(periDir, r), hn) < 0.0) nu = -nu;
                return nu;
            };
            double nu1 = trueAnom(m_detail.depPos);
            double nu2 = trueAnom(m_detail.arrPos);
            if (nu2 < nu1) nu2 += 2.0 * M_PI;

            ImVec2 prev{}; bool first = true;
            for (int k = 0; k <= 80; ++k) {
                double nu  = nu1 + (nu2 - nu1) * k / 80.0;
                double r_  = p_ / (1.0 + ecc * std::cos(nu));
                glm::dvec3 pos = r_ * (std::cos(nu) * periDir + std::sin(nu) * qDir);
                ImVec2 sp  = toScreen(pos);
                if (!first) dl->AddLine(prev, sp, kYellow, 1.5f);
                prev = sp; first = false;
            }
        }
    }

    // Body markers — dots on the real ellipses.
    ImVec2 depSc = toScreen(m_detail.depPos);
    ImVec2 arrSc = toScreen(m_detail.arrPos);
    dl->AddCircleFilled(depSc, 3.5f, IM_COL32( 60,140,255,255));
    dl->AddCircleFilled(arrSc, 3.5f, IM_COL32(200, 80, 50,255));
    dl->AddText({depSc.x + 4, depSc.y - 5}, IM_COL32( 60,140,255,200), "E");
    dl->AddText({arrSc.x + 4, arrSc.y - 5}, IM_COL32(200, 80, 50,200), "M");

    // v∞ arrow from departure position.
    if (vInfMag > 1e-6) {
        glm::dvec3 vDir = vInf / vInfMag;
        float aLen = 14.0f;
        ImVec2 tip = { depSc.x + static_cast<float>(vDir.x) * aLen,
                       depSc.y - static_cast<float>(vDir.y) * aLen };
        dl->AddLine(depSc, tip, kOrange, 1.5f);
        dl->AddCircleFilled(tip, 2.5f, kOrange);
    }
}
