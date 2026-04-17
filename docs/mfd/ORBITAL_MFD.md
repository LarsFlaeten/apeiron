# Orbital MFD (ORB)

Displays the spacecraft's orbital elements computed directly from the state vector.
Works for all orbit types: circular, elliptic, near-parabolic, and hyperbolic.

## Layout

```
REF • FRAME [• HYP]               TGT: <name>
─────────────────────────────────────────────
Alt/r   value    r/Alt   value (TGT)
Vel     value    Vel     value
Apo/Aph value    Apo/Aph value
Per/Phe value    Per/Phe value
t-AP    HH:MM    (skipped for TGT)
t-PE    HH:MM
Inc     value    Inc     value
RAAN    value    RAAN    value
Ecc     value    Ecc     value
ArgPe   value    ArgPe   value
h       value    (blank)
T       value    T       value
                   [orbit diagram →]
```

The orbit diagram occupies the right half of the MFD and rotates with right-drag.
Double-right-click resets to the top-down ecliptic view.

## Left-side Buttons

| Button | Action |
|--------|--------|
| **REF** | Open text input — type a NAIF body name (e.g. `EARTH`, `SUN`, `MARS`) to change the reference body. Press Enter to confirm, Escape to cancel. |
| **FRM** | Cycle through available reference frames (`ECLIPJ2000`, `J2000`). Resets the diagram view. |
| **TGT** | Open text input — type a spacecraft name (e.g. `ISS`) or a NAIF body name / integer ID (e.g. `MARS`, `4`, `VENUS`, `5`) to set a target orbit. Empty input + Enter clears the target. Press Escape to cancel. |

## Displayed Values

### Geocentric mode (REF = EARTH or any body with radius < 500,000 km)

| Field | Description | Unit |
|-------|-------------|------|
| **Alt** | Altitude above body surface | km |
| **Vel** | Orbital speed | km/s |
| **Apo** | Apoapsis altitude (– = no apoapsis / hyperbolic) | km |
| **Per** | Periapsis altitude | km |
| **t-AP** | Time to next apoapsis | HH:MM or H:MM:SS |
| **t-PE** | Time to next periapsis | HH:MM or H:MM:SS |
| **Inc** | Inclination (angle between orbit plane and reference plane) | ° |
| **RAAN** | Right Ascension of Ascending Node (from +X / vernal equinox) | ° |
| **Ecc** | Eccentricity | — |
| **ArgPe** | Argument of periapsis | ° |
| **h** | Specific angular momentum | km²/s |
| **T** | Orbital period (elliptic only) | min |
| **v∞** | Hyperbolic excess speed (hyperbolic orbits only, shown in orange) | km/s |

### Heliocentric mode (REF = SUN, body radius > 500,000 km)

Distances switch to AU and the period to days/years. Labels change:

| Field | Description | Unit |
|-------|-------------|------|
| **r** | Orbital radius from Sun centre | AU |
| **Vel** | Heliocentric speed | km/s |
| **Aph** | Aphelion radius | AU |
| **Phe** | Perihelion radius | AU |
| **T** | Orbital period (< 10 yr → days, ≥ 10 yr → years) | d / yr |

All angular elements (Inc, RAAN, Ecc, ArgPe) are measured in the ECLIPJ2000
ecliptic plane regardless of the selected frame.

## Reference Frame

The coordinate frame (toggled with **FRM**) rotates all vectors before computing
elements, so the inclination and RAAN are measured relative to that frame's
reference plane:

| Frame | Reference plane | Notes |
|-------|-----------------|-------|
| `ECLIPJ2000` | Ecliptic at J2000.0 | Default; matches the physics integrator |
| `J2000` | Earth mean equator at J2000.0 | Matches TLE / classical orbital element conventions |

## Target Orbit (TGT)

When a target is set, a second column of orbital elements appears in blue, and
the orbit diagram draws the target orbit (also blue) alongside the ship orbit.

**Spacecraft target** (type the spacecraft's name, e.g. `ISS`): shows ISS's
current orbital elements. Useful for proximity operations and rendezvous
planning.

**Body target** (type a NAIF name or integer, e.g. `MARS` or `4`): queries SPICE
each frame for that body's state relative to the current reference body and shows
its orbital elements. Examples:

- REF = `SUN`, TGT = `MARS` → compare your heliocentric orbit to Mars's orbit;
  watch Ecc and Aph/Phe converge as you fine-tune a transfer.
- REF = `SUN`, TGT = `EARTH` → see Earth's orbit receding as you coast outward.
- REF = `EARTH`, TGT = `MOON` → Moon's geocentric orbit for lunar-transfer planning.

Clear the target by pressing **TGT**, typing nothing, and pressing Enter.

## Coordinate System

All positions and velocities are in the **ECLIPJ2000** frame by default (the same
frame the physics integrator uses). Switching to **J2000** rotates vectors by
Earth's obliquity (~23.44°) to align with the equatorial plane.

The **h** (angular momentum) row is omitted in heliocentric mode to save space;
the value in geocentric mode has units km²/s.

## Implementation Notes

- Orbital elements are computed analytically from **r** and **v** — no iterative
  Kepler solver, so the display is well-behaved at all eccentricities including
  e → 1 (near-parabolic) and e > 1 (hyperbolic).
- The diagram scale freezes when e > 0.90 to prevent it from zooming out to
  nothing on high-eccentricity departure trajectories.
- The NAIF body target is re-queried from SPICE every frame, so it tracks the
  body's actual ephemeris position in real time.
