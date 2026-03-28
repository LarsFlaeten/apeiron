# Astro Library — Integration Reference

*Summary as of 2026-03-28. All 95 tests pass.*

Source: [github.com/LarsFlaeten/astro](https://github.com/LarsFlaeten/astro)
Integrated as a CMake submodule under `libs/astro/`.

---

## Overview

`astro` is a C++ astrodynamics support library wrapping NAIF SPICE N0067 for
planetary ephemerides and reference frames, with Keplerian orbital mechanics
and numerical integrators for translational and rotational dynamics.

All math types use **GLM double precision** (`glm::dvec3`, `glm::dquat`).
`astro::Vec3` = `glm::dvec3`, `astro::Quat` = `glm::dquat` — no conversion
at the Apeiron boundary.

---

## Dependencies

| Dep | Version | Notes |
|---|---|---|
| GLM | 1.0.1 | Header-only, FetchContent fallback |
| CSPICE | N0067 (Jan 2022) | Bundled static lib — place in `libraries/cspice/` (gitignored) |
| GTest | 1.14.0 | Tests only |
| cxxopts | 2.2.0 | Examples only |

---

## Spacecraft propagation

### Translational dynamics — `ODE`

Computes acceleration as a sum over N point-mass attractors:

```cpp
dxdt.v += (-a.GM / pow(R, 3)) * r;   // for each Attractor
```

Configure by adding `Attractor` structs (position + GM). Multiple attractors
work simultaneously (Earth + Moon + Sun). The force loop is `virtual` —
subclass and override `operator()` to add perturbations.

### Rotational dynamics — `RotODE`

Full Euler equations with inertia tensor:

```
L_dot = tau - ω × (I·ω)
```

Supports body-frame torques (thrusters) and global-frame torques (gravity
gradient). Quaternion kinematics integrated alongside angular velocity.

### Integrators

| Integrator | Type | Notes |
|---|---|---|
| RK1–RK4 | Fixed step, generic template | |
| RKF45 | Adaptive, tolerance-controlled | |
| RKF78 | Adaptive, higher accuracy | Good default for orbit propagation |
| PCDM | Quaternion-specific | Dedicated solver for `RotODE`; maintains normalisation |

**Note:** tolerances in RKF45/RKF78 are set via static calls — not reentrant.
Set tolerance once before propagation, not concurrently from multiple threads.

---

## Perturbations — what's missing

The `ODE::operator()` has an explicit TODO:

```cpp
// TODO: add perturbations:
// - Oblateness (J2 and higher zonal harmonics)
// - Atmospheric drag
// - Solar radiation pressure
// - Thruster forces
```

Current dynamics are **pure N-body point-mass gravity**. Implications for Apeiron:

| Perturbation | Impact | Priority |
|---|---|---|
| **J2 oblateness** | Dominant LEO perturbation (~1000× larger than J3). Orbit will precess wrong without it. A few lines to add by subclassing ODE. | High for LEO |
| **Thrust** | Not modelled. Any manoeuvre needs this. | High — add for spacecraft control |
| Atmospheric drag | Relevant below ~800 km. Simple exponential atmosphere model suffices for sim. | Medium |
| Solar radiation pressure | Only significant for high area-to-mass craft. | Low initially |

**Recommended approach:** subclass `ODE`, override `operator()`, call base
first then add perturbation terms. The architecture is designed for this.

---

## SPICE integration

SPICE is wrapped behind a singleton `Spice()` with a mutex for thread safety.

### Key API calls

```cpp
// Geometric state between two NAIF bodies in a given frame
Spice().getRelativeGeometricState(tgt_id, obs_id, et, state, frame);

// State relative to an Observer (supports aberration corrections)
Spice().getRelativeState(tgt_id, observer, et, state, abcorr);

// Position only (cheaper — avoids velocity computation)
Spice().getRelativePosition(tgt_id, obs_id, et, pos, abcorr, frame);
```

Aberration corrections: `None`, `LightTime`, `LightTimeStellar`,
`CNLightTime`, `CNLightTimeStellar`. For simulation, `None` (geometric)
is fastest and sufficient.

Results return as `astro::PosState` (position km, velocity km/s) in the
specified frame (default J2000).

### Calling raw SPICE safely

For SPICE functions astro doesn't expose, acquire the mutex first:

```cpp
{
    std::lock_guard<std::mutex> lock(Spice().mutex());
    // raw SPICE call here
}
Spice().checkError();
```

### Startup requirement

Load all needed kernels before any scene object queries SPICE. No lazy
loading or dependency tracking:

```cpp
Spice().loadKernel("naif0012.tls");   // LSK
Spice().loadKernel("de440s.bsp");     // SPK
Spice().loadKernel("pck00011.tpc");   // PCK
```

---

## Direct SPICE vs. astro wrappers

For body positions and states, **prefer `astro::Spice()`**:
- C++ exceptions instead of SPICE error codes
- Thread-safe (mutex managed internally)
- Results already in GLM types — no conversion needed
- Kernel loading tracked in `loadedKernels` list

Use raw SPICE only for functions astro doesn't expose (surface intercepts,
occultation finders, geometry routines), with the mutex pattern above.

---

## Notes for Apeiron integration

- `State::transform()` — correctly transforms position/velocity and
  orientation quaternion + angular velocity between frames (was broken,
  now fixed and tested).
- `ODE` and `RotODE` are architecturally separated — clean for spacecraft
  sim since translational and rotational dynamics have different natural
  time scales and can be integrated independently.
- Apeiron can and should extend the library as needed. Limitations in astro
  are not constraints — add perturbations, thrust models, etc. as Layer 5
  work proceeds.
