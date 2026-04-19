# Transfer MFD (XFER)

Interplanetary transfer planning display. Steps from porkchop window selection
through departure burn execution and heliocentric coast monitoring. Currently
hard-wired to an Earth → Mars transfer (NAIF 399 → 4) with the Sun as the
central body.

## Pages and Navigation Flow

```
[Page 0]  Porkchop plot
    │  Click cell → summary shown; press INFO (or double-click) to continue
    ▼
[Page 1]  Transfer detail     ← BACK returns to page 0
    │  BURN →
    ▼
[Page 2]  Departure burn      ← BACK returns to page 1
    │  CST →
    ▼
[Page 3]  Coasting / MCC      ← BACK returns to page 2
```

---

## Page 0 — Porkchop Plot

A colour-coded grid of total ΔV (departure + arrival) over departure date
(columns, x-axis) versus time-of-flight in days (rows, y-axis).

**Colour scale:** green = low ΔV → yellow → red = high ΔV.  
**Click** a cell to select it; the right side of the panel shows a short
summary (departure date, TOF, dV1, dV2, C3).  
**Double-click** or press **INFO** (right button 4) to open page 1.

Default window on first load: departure over the next 2 years, TOF 100–400 days,
80 × 60 grid cells.

### Page 0 Buttons

| Side | Button | Action |
|------|--------|--------|
| Left 0 | **DEP<** | Shift departure window 30 days earlier |
| Left 1 | **DEP>** | Shift departure window 30 days later |
| Left 2 | **TOF<** | Shift TOF range 30 days shorter |
| Left 3 | **TOF>** | Shift TOF range 30 days longer |
| Left 4 | **COMP** | Recompute the grid with current window settings |
| Right 0 | **WIN<** | Halve the departure window span (zoom in, same centre) |
| Right 1 | **WIN>** | Double the departure window span (zoom out) |
| Right 2 | **RNG<** | Halve the TOF range span |
| Right 3 | **RNG>** | Double the TOF range span |
| Right 4 | **INFO** | Open page 1 for the selected cell (visible only when a cell is selected) |

---

## Page 1 — Transfer Detail

Heliocentric orbit diagram (Sun at centre) showing:
- Earth orbit (dim blue)
- Mars orbit (dim red/brown)
- Transfer arc (yellow) from departure to arrival position
- Departure position **E**, arrival position **M** markers
- V∞ direction arrow at the departure point (orange)

Text panel (left side) shows:

| Field | Description |
|-------|-------------|
| `NOW` | Current simulation date |
| `DEP` | Departure date and T+ elapsed/remaining |
| `ARR` | Arrival date |
| `TOF` | Transfer time in days |
| `DV1` | Heliocentric departure ΔV = \|V∞\| (km/s) |
| `DV2` | Heliocentric arrival ΔV = \|v_arr − v_Mars\| (km/s) |
| `TOT` | DV1 + DV2 (km/s) |
| `C3` | Characteristic energy = \|V∞\|² (km²/s²) |
| `VINF LON / LAT` | Departure asymptote direction in ECLIPJ2000 (°) |
| `TMI alt` | Selected parking orbit altitude [ALT cycles options] |
| `Vcirc` | Circular speed at TMI altitude (km/s) |
| `Vperi` | Hyperbolic periapsis speed = √(C3 + 2μ/r) (km/s) |
| `dV-TMI` | Trans-Mars Injection burn ΔV = Vperi − Vcirc (km/s) |
| `inc` | Departure asymptote inclination above ecliptic (shown only when > 0.5°) |

The right half is occupied by the orbit diagram; drag right-mouse to rotate,
double-right-click to reset to top-down ecliptic view.

### Page 1 Buttons

| Side | Button | Action |
|------|--------|--------|
| Left 3 | **ALT** | Cycle parking orbit altitude: 400 / 600 / 1000 / 2000 km |
| Left 4 | **BACK** | Return to porkchop plot (page 0) |
| Right 4 | **BURN** | Open departure burn planning page (page 2) |

---

## Page 2 — Departure Burn

Geocentric view (Earth at centre) for planning the Trans-Mars Injection burn.
The diagram shows:
- Current parking orbit (green, with ship marker)
- Target orbit plane ghost (dim blue ring) — the plane that contains V∞
- AN / DN markers (ascending/descending node of current orbit w.r.t. target plane)
- **B** marker — optimal burn point
- V∞ direction arrow from Earth centre (orange)

The burn point **B** is the true anomaly at which a prograde burn produces an
escape hyperbola whose asymptote aligns with the required V∞ direction.

> **Implementation note:** the formula accounts for the hyperbolic deflection
> angle ν∞ = arccos(−1/e_hyp).  For C3 ≈ 9 km²/s² at 400 km altitude,
> e_hyp ≈ 1.16 and ν∞ ≈ 150°, placing B roughly 60° before the point where
> the velocity direction is prograde relative to V∞.

Text overlay (top-left, dark backing):

| Field | Description |
|-------|-------------|
| `NOW` | Current simulation date |
| `DEP` | Departure date and T+ |
| `V-inf` / `C3` | Departure V∞ magnitude (km/s) and C3 (km²/s²) |
| `CURRENT ORBIT` | i, Ω, ω, e, TA, altitude, SMA of the current parking orbit |
| `TARGET PLANE` | Required i and Ω; ΔI and plane error Perr |
| ` direct dV` | Direct plane-change cost at current orbit (km/s) and burn time |
| `BI-ELLIPTIC` | Plane-change costs at GEO / HEO / Lunar distance apoapsis (shown only when Perr > 0.5°) |
| `NODES` | AN and DN true anomalies and time-to-node |
| `BURN TA` | True anomaly of optimal burn point and altitude at that point |
| `TMI alt [ALT]` | Selected parking orbit radius for ΔV computation |
| ` dV-TMI` | Burn ΔV, Vperi, Vcirc |
| ` accel / burn dur` | Engine acceleration and total burn duration |
| ` T-to-burn-point` | Time from now to reaching the burn TA |
| ` T-to-ignition` | Time to start the burn (burn point minus half burn duration) |
| `dV-REMAINING` | Live C3-based remaining ΔV; turns green and shows **BURN COMPLETE** when C3 ≥ C3_req |

### Page 2 Buttons

| Side | Button | Action |
|------|--------|--------|
| Left 3 | **ALT** | Cycle parking orbit altitude: 400 / 600 / 1000 / 2000 km |
| Left 4 | **BACK** | Return to transfer detail (page 1) |
| Right 4 | **CST** | Open coasting / MCC view (page 3) |

---

## Page 3 — Coasting / MCC

Heliocentric diagram updated every frame showing the ship's progress along the
planned transfer arc:

- Earth orbit and Mars orbit (dim background rings)
- Planned Lambert arc (yellow, from departure **D** to arrival **A**)
- MCC arc (cyan) — the corrected Lambert arc from the current position to the
  arrival point; suppressed when the Lambert solution is degenerate (ecliptic
  inclination > 30°) or when within 1 day of arrival
- Current positions: **E** (Earth), **M** (Mars), **S** (ship)
- Sun at centre

Text overlay:

| Field | Description |
|-------|-------------|
| `DEP` / `ARR` | Planned departure and arrival dates |
| `PROG` | ASCII progress bar, percentage, elapsed and remaining days |
| `HELIO v / r` | Current heliocentric speed (km/s) and radius (AU) |
| ` dE / dM` | Distance to Earth and distance to Mars (Mkm) |
| `MCC dV` | Mid-course correction ΔV required right now (km/s); green = on-nominal |
| ` ON NOMINAL TRAJECTORY` | Shown when MCC dV < 1 m/s |
| `ARRIVAL dV-arr` | Planned arrival ΔV from the Lambert solution (shown at T-0) |

### Page 3 Buttons

| Side | Button | Action |
|------|--------|--------|
| Left 4 | **BACK** | Return to departure burn page (page 2) |

---

## Coordinate System

All heliocentric quantities use **ECLIPJ2000** (ecliptic plane at J2000.0),
consistent with the physics integrator.  Geocentric quantities (page 2) also
use ECLIPJ2000; the departure asymptote direction angles (VINF LON/LAT) are
therefore ecliptic longitude and latitude.

---

## Implementation Notes

- The porkchop grid is computed synchronously on **COMP** press; each cell
  runs Izzo's Lambert solver (N=0 revolutions, prograde).
- The optimal burn true anomaly accounts for the hyperbolic deflection angle
  (asymptote ≠ velocity direction).  The eccentricity of the escape hyperbola
  `e_hyp = 1 + C3·r_park / μ_Earth` determines ν∞ = arccos(−1/e_hyp); the
  correct burn TA is `φ_target − ν∞` where φ_target is the in-plane angle of
  V∞ in the (periDir, qDir) frame.
- The dV-remaining indicator uses the conserved quantity C3 = v² − 2μ/r
  rather than instantaneous speed, so it stays at zero once the burn is
  complete regardless of where in the orbit the spacecraft is.
- The MCC Lambert re-solve runs every frame during coasting.  Solutions with
  ecliptic inclination > 30° are suppressed (Lambert singularity near the
  departure geometry early in the coast).
