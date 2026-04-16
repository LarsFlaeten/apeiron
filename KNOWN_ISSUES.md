# Known Issues

## Earth mesh shows backfaces (mirrored texture) at day/night terminator

**Severity:** Cosmetic  
**Reproducible:** Yes — consistently once per orbit at high time warp (×1000+)

### Symptoms
- Earth surface texture appears mirrored (backfaces rendered) for ~1 frame
- Occurs when the spacecraft crosses from the dark side to the bright side (terminator)
- Secondary symptom: occasional single-frame glitch in the rendered Earth position

### Root cause (suspected, not yet confirmed)
The artifact lasts for multiple real-time frames (long enough to screen-grab), so it is NOT
a single-frame origin snap.  The spacecraft is inside the affected state for the full
duration of the terminator crossing.

Most likely candidates:
1. **Sustained depth-test failure at the terminator** — the Earth surface and atmosphere
   shell fragment depths are close at this Sun angle, causing the depth test to incorrectly
   let the Earth's backfaces win for several frames.
2. **Day/night rendering mode transition** — if the Earth pipeline switches between a lit
   and a night-side pass near the terminator, a pipeline state or descriptor-set swap may
   leave back-face culling in the wrong mode for the transition period.
3. **Floating origin kept in bad state** — the render origin may be oscillating around a
   threshold rather than snapping once, keeping Earth's render-space position wrong while
   the Sun-relative condition is true.

The secondary "occasional Earth position glitch" is consistent with either #1 or #3.

### Fix
Requires investigation of the Earth rendering pipeline and/or `Scene` origin update logic.
Deferred until after the interplanetary transfer work.  
Suspected files: `libs/universe/src/Scene.cpp`, `libs/render/src/AtmospherePipeline.cpp`,
Earth mesh draw call in `apps/apeiron/main.cpp`.
