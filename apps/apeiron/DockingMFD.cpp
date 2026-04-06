#include "DockingMFD.h"

#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <cstdio>
#include <numbers>

// ---------------------------------------------------------------------------
void DockingMFD::setCamNodes(const std::vector<std::string>& nodes)
{
    m_camNodes = nodes;
    m_camIdx   = 0;
}

const std::string& DockingMFD::preferredCamNode() const
{
    static const std::string kEmpty;
    if (m_camNodes.empty()) return kEmpty;
    return m_camNodes[m_camIdx];
}

// ---------------------------------------------------------------------------
void DockingMFD::autoSelectCam(const std::string& portLabel)
{
    // 1. Try exact suffix match: "cam_dock_<portLabel>"
    for (int i = 0; i < static_cast<int>(m_camNodes.size()); ++i) {
        const auto& n = m_camNodes[i];
        if (n.size() > portLabel.size() &&
            n.compare(n.size() - portLabel.size(), portLabel.size(), portLabel) == 0) {
            m_camIdx = i;
            return;
        }
    }
    // 2. Any cam whose name contains "dock"
    for (int i = 0; i < static_cast<int>(m_camNodes.size()); ++i) {
        if (m_camNodes[i].find("dock") != std::string::npos) {
            m_camIdx = i;
            return;
        }
    }
    // 3. Leave current selection as-is
}

// ---------------------------------------------------------------------------
void DockingMFD::update(
    const Spacecraft&                                ship,
    const std::vector<std::vector<DockPort>>&        scPorts,
    size_t                                           playerIdx,
    const NavState&                                  nav,
    const std::vector<std::unique_ptr<Spacecraft>>&  spacecraft)
{
    m_hasTarget = false;
    m_hasPort   = false;

    if (nav.dockTgtIdx < 0 || nav.dockTgtIdx >= static_cast<int>(spacecraft.size()))
        return;

    m_hasTarget = true;
    auto& tgt = *spacecraft[static_cast<size_t>(nav.dockTgtIdx)];

    // ---- Player port (use first port; Orion has one probe) ----
    const auto& playerPorts = (playerIdx < scPorts.size()) ? scPorts[playerIdx]
                                                            : std::vector<DockPort>{};
    if (playerPorts.empty()) {
        // No player port — still show range/velocity to target CoM
        glm::dvec3 sep    = tgt.position() - ship.position();
        m_rangeM          = static_cast<float>(glm::length(sep) * 1000.0);
        glm::dvec3 relVel = (ship.velocity() - tgt.velocity()) * 1000.0;
        // Closure along separation vector
        if (m_rangeM > 1e-3f) {
            glm::dvec3 sepHat = sep / glm::length(sep);
            m_closureMs = static_cast<float>(glm::dot(relVel, sepHat));
        }
        return;
    }

    // Auto-select cam when the selected port changes
    if (nav.dockPortIdx != m_lastPortIdx) {
        m_lastPortIdx = nav.dockPortIdx;
        // Determine port label for cam lookup
        const auto& tgtPorts = (nav.dockTgtIdx < static_cast<int>(scPorts.size()))
                               ? scPorts[static_cast<size_t>(nav.dockTgtIdx)]
                               : std::vector<DockPort>{};
        if (nav.dockPortIdx >= 0 && nav.dockPortIdx < static_cast<int>(tgtPorts.size()))
            autoSelectCam(tgtPorts[nav.dockPortIdx].label);
        else
            autoSelectCam("");
    }

    const DockPort& pPort = playerPorts[0];

    // Check if a target port is selected
    const auto& tgtPorts = (nav.dockTgtIdx < static_cast<int>(scPorts.size()))
                           ? scPorts[static_cast<size_t>(nav.dockTgtIdx)]
                           : std::vector<DockPort>{};
    if (nav.dockPortIdx < 0 || nav.dockPortIdx >= static_cast<int>(tgtPorts.size())) {
        // No target port — show range to CoM only
        glm::dvec3 sep    = tgt.position() - ship.position();
        m_rangeM          = static_cast<float>(glm::length(sep) * 1000.0);
        glm::dvec3 relVel = (ship.velocity() - tgt.velocity()) * 1000.0;
        if (m_rangeM > 1e-3f) {
            glm::dvec3 sepHat = sep / glm::length(sep);
            m_closureMs = static_cast<float>(glm::dot(relVel, sepHat));
        }
        return;
    }

    m_hasPort = true;
    const DockPort& tPort = tgtPorts[static_cast<size_t>(nav.dockPortIdx)];

    // ---- Player port world frame ----
    glm::dquat pAtt = ship.attitude();
    glm::dvec3 pAxisX  = pAtt * glm::dvec3(pPort.axisX);           // outward (toward ISS)
    glm::dvec3 pAxisZ  = pAtt * glm::dvec3(pPort.axisZ);           // roll reference
    glm::dvec3 pAxisY  = glm::normalize(glm::cross(pAxisZ, pAxisX)); // "left" in port frame
    // Reorthogonalise Z in case it wasn't perfectly perpendicular
    pAxisZ = glm::normalize(glm::cross(pAxisX, pAxisY));

    glm::dvec3 pPortPos = ship.position()
                        + pAtt * glm::dvec3(pPort.posM) * 1e-3;    // m → km

    // ---- Target port world frame ----
    glm::dquat tAtt = tgt.attitude();
    glm::dvec3 tAxisX   = tAtt * glm::dvec3(tPort.axisX);
    glm::dvec3 tAxisZ   = tAtt * glm::dvec3(tPort.axisZ);
    glm::dvec3 tPortPos = tgt.position()
                        + tAtt * glm::dvec3(tPort.posM) * 1e-3;

    // ---- Separation (player port → target port, km) ----
    glm::dvec3 sep = tPortPos - pPortPos;
    m_rangeM = static_cast<float>(glm::length(sep) * 1000.0);

    // ---- Lateral offset in player port YZ plane (m) ----
    m_latY = static_cast<float>(glm::dot(sep, pAxisY) * 1000.0);
    m_latZ = static_cast<float>(glm::dot(sep, pAxisZ) * 1000.0);

    // ---- Closure rate along player port X axis (m/s, +ve = approaching) ----
    glm::dvec3 relVelMs = (ship.velocity() - tgt.velocity()) * 1000.0;
    m_closureMs = static_cast<float>(glm::dot(relVelMs, pAxisX));

    // ---- Attitude errors ----
    // When perfectly docked: pAxisX = -tAxisX  (they face each other)
    // Express tAxisX in player port frame to decompose into yaw/pitch
    double tX_x = glm::dot(tAxisX, pAxisX);    // should be ≈ -1 when aligned
    double tX_y = glm::dot(tAxisX, pAxisY);    // yaw component
    double tX_z = glm::dot(tAxisX, pAxisZ);    // pitch component
    m_yawErrDeg   = static_cast<float>(glm::degrees(std::atan2( tX_y, -tX_x)));
    m_pitchErrDeg = static_cast<float>(glm::degrees(std::atan2( tX_z, -tX_x)));

    // Roll: project tAxisZ onto the plane ⊥ to pAxisX, measure against pAxisZ
    glm::dvec3 tZ_perp = tAxisZ - glm::dot(tAxisZ, pAxisX) * pAxisX;
    if (glm::length(tZ_perp) > 1e-9) {
        tZ_perp = glm::normalize(tZ_perp);
        double cosR = glm::clamp(glm::dot(tZ_perp, pAxisZ), -1.0, 1.0);
        double sinR = glm::dot(tZ_perp, pAxisY);
        m_rollErrDeg = static_cast<float>(glm::degrees(std::atan2(sinR, cosR)));
    } else {
        m_rollErrDeg = 0.0f;
    }

    // ---- Dynamic indicator scale ----
    m_fullScaleM = m_rangeM > 1000.0f ? 1000.0f :
                   m_rangeM >  200.0f ?  100.0f :
                   m_rangeM >   20.0f ?   10.0f :
                                           2.0f;
}

// ---------------------------------------------------------------------------
const char* DockingMFD::leftLabel(int slot) const
{
    if (slot == 0) return "CAM";
    return "";
}

void DockingMFD::onLeft(int slot)
{
    if (slot == 0 && !m_camNodes.empty())
        m_camIdx = (m_camIdx + 1) % static_cast<int>(m_camNodes.size());
}

// ---------------------------------------------------------------------------
void DockingMFD::render(ImDrawList* dl, ImVec2 origin, ImVec2 size)
{
    const ImU32 kGreen  = IM_COL32(  0, 210,  75, 210);
    const ImU32 kDim    = IM_COL32(  0, 140,  50, 150);
    const ImU32 kCyan   = IM_COL32(  0, 200, 255, 200);
    const ImU32 kYellow = IM_COL32(220, 200,   0, 220);
    const ImU32 kRed    = IM_COL32(255,  80,  60, 220);

    const float lh  = ImGui::GetTextLineHeight();
    const float pad = 4.0f;
    char buf[64];

    float y = origin.y + pad;

    // ---- Camera label (top, centred) ----
    if (!m_camNodes.empty()) {
        const char* camName = m_camNodes[static_cast<size_t>(m_camIdx)].c_str();
        ImVec2 tsz = ImGui::CalcTextSize(camName);
        dl->AddText({ origin.x + (size.x - tsz.x) * 0.5f, y }, kDim, camName);
    }
    y += lh + pad;

    // ---- No target ----
    if (!m_hasTarget) {
        const char* msg = "NO TARGET";
        ImVec2 tsz = ImGui::CalcTextSize(msg);
        dl->AddText({ origin.x + (size.x - tsz.x) * 0.5f,
                      origin.y + size.y * 0.5f - lh * 0.5f }, kDim, msg);
        return;
    }

    // ---- Range + closure velocity ----
    {
        // Range: green = within capture threshold, yellow = close, red = far
        ImU32 rngCol = (m_rangeM <= m_threshRangeM)          ? kGreen  :
                       (m_rangeM <= m_threshRangeM * 10.0f)  ? kYellow : kRed;
        if (m_rangeM < 1000.0f)
            std::snprintf(buf, sizeof(buf), "RNG %6.1f m", m_rangeM);
        else
            std::snprintf(buf, sizeof(buf), "RNG %6.3f km", m_rangeM * 1e-3f);
        dl->AddText({ origin.x + pad, y }, rngCol, buf);

        // Velocity: green = within threshold, yellow = moderate, red = too fast or opening
        const float absVel = std::abs(m_closureMs);
        ImU32 velCol = (absVel <= m_threshClosureMs)         ? kGreen  :
                       (absVel <= m_threshClosureMs * 3.0f)  ? kYellow : kRed;
        if (m_closureMs < 0.0f) velCol = kRed;  // opening = always red
        std::snprintf(buf, sizeof(buf), "%+.2f m/s", m_closureMs);
        ImVec2 vsz = ImGui::CalcTextSize(buf);
        dl->AddText({ origin.x + size.x - vsz.x - pad, y }, velCol, buf);
    }
    y += lh + pad;

    // ---- Lateral indicator ----
    // Takes up the remaining height minus a three-row footer: P/Y, R, capture phase.
    const float footerH = 3.0f * (lh + pad);
    const float availH  = size.y - (y - origin.y) - footerH;
    const float sqSide  = std::max(20.0f, std::min(size.x - 2.0f * pad, availH));
    const ImVec2 sqTL   { origin.x + (size.x - sqSide) * 0.5f, y };
    const ImVec2 sqBR   { sqTL.x + sqSide, sqTL.y + sqSide };
    const float  cx     = sqTL.x + sqSide * 0.5f;
    const float  cy     = sqTL.y + sqSide * 0.5f;
    const float  hs     = sqSide * 0.5f;   // half-side in pixels

    if (!m_hasPort) {
        // Just a dim placeholder box
        dl->AddRect(sqTL, sqBR, kDim, 0.0f, 0, 1.0f);
        const char* msg = "PORT?";
        ImVec2 tsz = ImGui::CalcTextSize(msg);
        dl->AddText({ cx - tsz.x * 0.5f, cy - tsz.y * 0.5f }, kDim, msg);
    } else {
        // Scale rings at 25% and 75% of half-side
        const float r1px = hs * 0.25f;
        const float r2px = hs * 0.75f;
        const float r1m  = m_fullScaleM * 0.25f;
        const float r2m  = m_fullScaleM * 0.75f;

        dl->AddRect(sqTL, sqBR, kDim, 0.0f, 0, 1.0f);
        dl->AddCircle({ cx, cy }, r1px, kDim, 0, 1.0f);
        dl->AddCircle({ cx, cy }, r2px, kDim, 0, 1.0f);

        // Ring labels
        std::snprintf(buf, sizeof(buf), "%.0fm", r1m);
        dl->AddText({ cx + r1px + 2.0f, cy - lh * 0.5f }, kDim, buf);
        std::snprintf(buf, sizeof(buf), "%.0fm", r2m);
        dl->AddText({ cx + r2px + 2.0f, cy - lh * 0.5f }, kDim, buf);

        // Crosshair (broken at centre ±8 px)
        constexpr float kGap = 8.0f;
        dl->AddLine({ sqTL.x, cy }, { cx - kGap, cy }, kDim, 1.0f);
        dl->AddLine({ cx + kGap, cy }, { sqBR.x, cy }, kDim, 1.0f);
        dl->AddLine({ cx, sqTL.y }, { cx, cy - kGap }, kDim, 1.0f);
        dl->AddLine({ cx, cy + kGap }, { cx, sqBR.y }, kDim, 1.0f);

        // Target port dot: map (latY, latZ) → pixel offset
        // +latY = right on screen, +latZ = up on screen (negate for ImGui y)
        float dxPx = (m_latY / m_fullScaleM) * hs;
        float dyPx = -(m_latZ / m_fullScaleM) * hs;

        float dotDist = std::sqrt(dxPx * dxPx + dyPx * dyPx);
        bool  offScale = dotDist > hs * 0.95f;
        if (offScale) {
            float s = hs * 0.95f / dotDist;
            dxPx *= s;  dyPx *= s;
        }
        ImVec2 dotPos { cx + dxPx, cy + dyPx };
        ImU32  dotCol = offScale ? kYellow : kCyan;

        dl->AddCircleFilled(dotPos, 4.0f, dotCol);
        if (!offScale)
            dl->AddCircle(dotPos, 9.0f, dotCol, 0, 1.5f);

        // Full-scale label in top-right corner of indicator
        std::snprintf(buf, sizeof(buf), "±%.0fm", m_fullScaleM);
        ImVec2 labSz = ImGui::CalcTextSize(buf);
        dl->AddText({ sqBR.x - labSz.x - 2.0f, sqTL.y + 2.0f }, kDim, buf);
    }

    y = sqBR.y + pad;

    // ---- Attitude errors (P / Y / R) ----
    if (m_hasPort) {
        // Colour: green if all within 2°, yellow otherwise
        const float maxPY = m_threshAttErrDeg * 0.4f;  // pitch/yaw tighter than combined
        const float maxR  = m_threshAttErrDeg;
        bool aligned = std::abs(m_pitchErrDeg) < maxPY
                    && std::abs(m_yawErrDeg)   < maxPY
                    && std::abs(m_rollErrDeg)  < maxR;
        bool tooFar  = std::abs(m_pitchErrDeg) > maxPY * 3.0f
                    || std::abs(m_yawErrDeg)   > maxPY * 3.0f
                    || std::abs(m_rollErrDeg)  > maxR  * 3.0f;
        ImU32 attCol = aligned ? kGreen : (tooFar ? kRed : kYellow);

        std::snprintf(buf, sizeof(buf), "P %+5.1f°  Y %+5.1f°",
                      m_pitchErrDeg, m_yawErrDeg);
        ImVec2 tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ origin.x + (size.x - tsz.x) * 0.5f, y }, attCol, buf);
        y += lh + pad;

        std::snprintf(buf, sizeof(buf), "R %+5.1f°", m_rollErrDeg);
        tsz = ImGui::CalcTextSize(buf);
        dl->AddText({ origin.x + (size.x - tsz.x) * 0.5f, y }, attCol, buf);
        y += lh + pad;
    }

    // ---- Capture phase status ----
    {
        const char* label = nullptr;
        ImU32 col = kDim;
        switch (m_capturePhase) {
            case CapturePhase::Armed:       label = "ARMED";        col = kCyan;   break;
            case CapturePhase::SoftCapture: label = "SOFT CAPTURE"; col = kGreen;  break;
            case CapturePhase::HardCapture: label = "HARD CAPTURE"; col = kYellow; break;
            default: break;
        }
        if (label) {
            ImVec2 tsz = ImGui::CalcTextSize(label);
            dl->AddText({ origin.x + (size.x - tsz.x) * 0.5f, y }, col, label);
        }
    }
}
