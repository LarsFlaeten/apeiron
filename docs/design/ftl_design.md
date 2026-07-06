# Apeiron FTL Architecture

Design document for adding faster-than-light travel to Apeiron while preserving the
simulator's physics-first character. Summarizes the theoretical background, the design
space explored in fiction, and a concrete recommended architecture with implementation
phases.

**Status:** Draft / design input for coding agent
**Prerequisites already in Apeiron:** state-vector propagation, ephemerides, Lambert
solver (Izzo), porkchop plots, patched conic planning, mission planning MFD, SCAS.

---

## 1. Design goals

1. FTL must not obsolete the existing orbital mechanics stack. Jumps solve *distance*,
   not *velocity matching*. Arrival and departure remain rendezvous/encounter problems
   solvable with the existing Lambert/porkchop tooling.
2. Prefer mechanisms with a defensible GR pedigree (Morris–Thorne / Visser traversable
   wormholes) over pure handwaves. Where handwaves are unavoidable, replace missing
   physics with explicit, consistent engineering constraints.
3. Conservation laws are gameplay. Momentum and mass-energy bookkeeping on wormhole
   mouths should be simulated, not ignored.
4. Chronology protection is assumed (mouths kept co-moving; no time-machine regimes).

## 2. Theoretical baseline: Visser-style traversable wormholes

Key results from GR literature (Morris–Thorne 1988; Visser, *Lorentzian Wormholes:
From Einstein to Hawking*):

- A wormhole has **two mouths**, each a physical object with its own worldline: a
  position, velocity, mass, and orientation. Mouths orbit, can be towed, and appear
  in ephemerides like any other body.
- **Traversal rule:** velocity is continuous through the throat *in mouth-relative
  terms*. A ship's velocity relative to the exit mouth equals its velocity relative
  to the entry mouth, rotated by the throat's fixed frame mapping:

  ```
  v_ship_out = R_throat * (v_ship_in - v_mouthA) + v_mouthB
  q_ship_out = R_throat * q_ship_in            (attitude, as quaternion composition)
  ```

  There is no absolute frame; "keeping your Sol velocity" is meaningless. What is
  preserved is the mouth-relative state vector.
- **Entry orientation does not select the destination.** The throat is topological.
  Pointing the entry mouth "at the target star" has no physical meaning.
- **Conservation bookkeeping (Visser):** when a ship of momentum p and mass-energy m
  transits A→B:
  - Mouth A absorbs the ship's momentum (recoils by +p) and gains mass-energy +m.
  - Mouth B recoils by −p and loses mass-energy −m.
  - Consequences to simulate: heavy one-way traffic perturbs mouth orbits and drains
    the exit mouth's mass budget. An exit mouth cannot emit more mass-energy than it
    has. Route operators must periodically rebalance (reverse traffic, mass shipments,
    station-keeping burns).
- **Mouth delivery problem:** the far mouth must be physically transported to the
  destination system (sub-light slowboat). The delivery vehicle pays the interstellar
  velocity mismatch (typically 20–60 km/s of stellar peculiar velocity) once, up
  front, by decelerating the mouth into a sensible local orbit. Travelers thereafter
  inherit that orbit for free.
- **Chronology protection:** if mouths acquire significantly different velocity
  histories, the inter-mouth time offset grows toward time-machine regimes (Thorne).
  Design rule: keep paired mouths co-moving; expose a "desynchronization" scalar if
  we ever want it as a failure mode, but do not simulate causality violation.

## 3. Design space (survey of fictional models)

| Model | Exit determined by | Velocity at exit | Alignment constraint | Orbital mechanics preserved? |
|---|---|---|---|---|
| **Fixed mouths (Visser / Hamilton Commonwealth gateways)** | Physical far mouth (delivered once) | Mouth-relative velocity preserved | None (topological) | Yes — rendezvous with mouths |
| **Ballistic self-jump (Hamilton Night's Dawn ZTT)** | Departure pointing vector + range | Own-frame velocity carried through, translated | Hard: jump axis must align with target star; low gravitational potential required; orbit-dependent jump windows | Yes — arrival is a real encounter problem |
| **Continuous-traversal / dynamic wormhole (Hamilton *Second Chance*; Alcubierre-like)** | Continuous steering | Own-frame velocity preserved; pseudo-velocity during transit | Must enter/exit in weak field (system fringe) | Yes — behaves as speed multiplier on a normal trajectory |
| **Remote aperture projection (Commonwealth CST Exploratory Division)** | Generator-side machinery projects/steers far end, incl. surface anchoring | Handled invisibly by generator | Max range per hop; continuous power cost; aperture size cost | **No** — deletes state vectors for travelers. Anti-goal for Apeiron unless restricted to space-based apertures |
| **Velocity zeroed at destination (Elite-style)** | Convention | Zeroed relative to local star | None | No — implies preferred frame, momentum dump |
| **Computed jump points (Niven/Pournelle Alderson tramlines)** | Stellar physics (mass, fusion state of star pairs) | Carried through | Jump points exist only where physics says | Yes — jump geography emerges from simulation data |
| **Fringe arrival + velocity dump (Cherryh Alliance-Union)** | Mass points (stars, brown dwarfs) | Enormous residual velocity; "Vdump" is a named operational procedure | Jump between sufficient masses | Yes — arrival ops dominated by braking |

Additional reference ideas worth stealing:
- **FTL as field property of space** (Vinge zones): drive availability varies by region.
- **FTL with environmental footprint** (Liu Cixin curvature drive): use degrades local
  space and leaves detectable trails — natural SCAS integration (FTL wakes as sensor
  contacts).
- **No-FTL relativistic honesty** (Reynolds, Haldeman, Anderson): if relativistic
  sub-light regimes are ever added, time dilation bookkeeping is the story.

## 4. Recommended architecture

### 4.1 Phase 1 — Fixed wormhole mouths (Visser model)

Cheapest to implement against existing machinery; richest emergent behavior.

**Data model**

```
struct WormholeMouth {
    BodyId        body_id;        // participates in ephemerides like any body
    StateVector   state;          // r, v in local system frame
    Quaterniond   orientation;
    double        mass;           // kg — mass-energy reservoir
    double        mass_min;       // cannot emit below this (throat collapse threshold)
    MouthId       paired_mouth;
    Mat3d         R_throat;       // fixed frame rotation entry->exit
    double        throat_radius;  // aperture constraint on ship size
    SystemId      system;
};
```

**Traversal (instantaneous event)**

```
on_transit(ship, mouthA, mouthB):
    v_rel   = ship.v - mouthA.v
    ship.r  = mouthB.r + R_throat * (ship.r - mouthA.r)   // offset within aperture
    ship.v  = mouthB.v + R_throat * v_rel
    ship.q  = R_throat_quat * ship.q

    // Visser conservation bookkeeping
    p = ship.mass * v_rel
    mouthA.v += p / mouthA.mass;     mouthA.mass += ship.mass
    mouthB.v -= p / mouthB.mass;     mouthB.mass -= ship.mass
    assert(mouthB.mass > mouthB.mass_min)   // else transit refused / throat event
```

**Gameplay/simulation consequences to implement**
- Approach to a mouth is a standard rendezvous problem (existing MFD tooling).
- Mouths need station-keeping; traffic history perturbs their orbits.
- One-way traffic drains exit-mouth mass; expose in route economics/telemetry.
- Mouths appear in SCAS as tracked objects; transit events are observable.
- Destination mouth parking orbits are a design decision per route (e.g., NRHO
  around a moon, heliocentric at fringe, Lagrange points).

**Explicit non-features:** no time-machine regimes (chronology protection assumed);
entry orientation never selects destination.

### 4.2 Phase 2 — Portable jump drive (momentum-conserving convention)

If/when a ship-carried drive is added, use the convention: **the jump translates
position; the ship keeps its own-frame velocity, re-expressed at the destination.**
Any desired frame change costs delta-v (propellant/energy) exactly as a burn would.

```
on_jump(ship, target_point, target_frame):
    ship.r = target_point
    ship.v = frame_remap(ship.v, origin_frame, target_frame)  // galactic-frame carry-over
    // no free velocity change; arrival encounter solved with Lambert as usual
```

- Interstellar jumps therefore arrive with the full stellar peculiar-velocity
  mismatch (~20–60 km/s) to be paid by the ship. This keeps drive power / delta-v
  budgets meaningful.
- Optional Night's Dawn-flavored constraints (recommended, both cheap):
  - **Alignment gate:** jump permitted only when the ship's jump axis is within a
    cone of the target direction → orbit-dependent jump windows fall out naturally.
  - **Potential gate:** local gravitational potential must be below a threshold
    (system-fringe / high-orbit jumps only).

### 4.3 Phase 3 (optional) — Continuous-traversal drive

*Second Chance*-style dynamic wormhole: pseudo-velocity multiplier along a steered
trajectory, weak-field entry/exit constraint, transit duration = distance /
pseudo-velocity. Easiest model to make feel physical; trivially compatible with the
existing propagator (it is just a modified integrator regime). Consider FTL wake as
a SCAS-detectable signature (Liu-style) for detection gameplay.

### 4.4 Explicit anti-goals

- Surface-anchored, remotely projected apertures (CST model): deletes orbital
  mechanics for travelers. If aperture projection is ever wanted, restrict to
  space-based exits that become real objects with ephemerides once instantiated.
- Velocity-zeroed-at-destination jumps: preferred frame + unexplained momentum dump.

## 5. Open questions

- Units/precision: interstellar coordinates need a galactic frame layer above the
  per-system frames; decide on the frame hierarchy and where R_throat is defined.
- Mouth mass scales: pick masses large enough that per-transit recoil is small but
  cumulative drift is measurable over campaign timescales.
- Throat event on mass exhaustion: refuse transit vs. destructive collapse.
- Does SCAS propagate track data across mouths (network effectively links the two
  systems' traffic pictures)?
- Time handling: single simulation clock across systems (chronology protection makes
  this consistent by construction).

## 6. References

- M. Morris & K. Thorne, "Wormholes in spacetime and their use for interstellar
  travel" (Am. J. Phys., 1988) — traversability conditions.
- M. Visser, *Lorentzian Wormholes: From Einstein to Hawking* — mouth dynamics,
  conservation bookkeeping, thin-shell wormholes.
- K. Thorne et al., wormhole time-machine papers — why mouths must stay co-moving.
- K. Thorne, *The Science of Interstellar* — worked example of constraint-driven
  wormhole design.
- Fiction survey: P.F. Hamilton (*Night's Dawn*, Commonwealth Saga), Niven &
  Pournelle (*The Mote in God's Eye*), C.J. Cherryh (Alliance-Union), J.S.A. Corey
  (*The Expanse*), V. Vinge (*A Fire Upon the Deep*), Liu Cixin (*Death's End*).