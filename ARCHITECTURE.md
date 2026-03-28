# Apeiron — Architecture & Strategy

> *Apeiron* (ἄπειρον): Anaximander's word for the boundless, unlimited fundamental substance of the universe. A physics-correct, unlimited space simulator.

---

## North Star 🌟

**Fly a spacecraft whose onboard computer (OBC) is a running SPARC/LEON3 processor emulation.**

The LEON3 is the processor architecture used in real ESA spacecraft (Rosetta, Mars Express, BepiColombo, etc.). The north star for Apeiron is a hardware-in-the-loop (HIL) simulator where:

- The spacecraft exists in Apeiron's physically correct universe
- The OBC slot in the spacecraft module system is filled by an actual LEON3 emulator
- The sim feeds the OBC real sensor data — position, velocity, attitude, engine state — over an emulated bus
- The OBC runs real GNC (Guidance, Navigation & Control) software — compiled C or bare-metal SPARC — and computes burns, attitude corrections, and orbital maneuvers
- Apeiron advances physics based on the actuator commands the OBC actually outputs

This is not just a cool demo. It is a proper **Hardware-in-the-Loop simulator**, exactly the class of tool used to validate real spacecraft software before launch. The interface between emulator and sim is a clean HIL boundary: sensor data in, actuator commands out.

Possible software stacks to run on the emulated LEON3:
- Bare-metal GNC code (SPARC assembly or C)
- [RTEMS](https://www.rtems.org/) — the open source RTOS used on real ESA missions
- ESA open source flight software components

The LEON3 emulator project: [`github.com/LarsFlaeten/...`] — to be integrated as a spacecraft OBC module in Layer 5.

---

## Vision

A physically correct space simulator with an effectively unlimited universe — correct planetary positions, realistic rendering, and the ability to eventually visit other star systems. Similar in spirit to Orbiter, but modern, modular, and built on correct physics throughout.

Key principles:
- **Physics first** — correct ephemeris, correct orbital mechanics, correct lighting
- **Unlimited scale** — solar system to interstellar, handled architecturally
- **Modular** — clean separation between physics, scene management, and rendering
- **Instructive** — building core systems ourselves where that adds value and understanding

---

## The Core Challenge: Floating Point Scale

The fundamental problem in a universe-scale simulator is **floating point precision**. A 64-bit double gives ~15 significant digits. At 1 AU (≈1.5×10¹¹ m) that yields millimeter precision — but naively mixing planetary and spacecraft coordinates in one render scene destroys precision catastrophically.

This must be solved architecturally from day one. The chosen approach:

- **64-bit simulation, 32-bit render** — all physics and position data in `double`; translated to camera-relative `float` just before the GPU
- **Origin recentering** — the floating origin is kept near the camera each frame; all world-space positions are offsets from this origin
- **Hierarchical coordinate frames** — objects live in local frames (planet-relative, ship-relative, etc.) and are only transformed to world space at render time

---

## Layered Architecture

### Layer 1 — Universe & Physics Backend (`libs/astro`, `libs/universe`)

Pure math/physics, no graphics dependency. Independently testable against known data.

- **Time system** — UTC/TDB/TT, Julian dates, SPICE time conversion
- **Coordinate frames** — ICRF, body-fixed, ecliptic, topocentric
- **Ephemeris** — SPICE kernels (primary), VSOP87 (lightweight fallback)
- **Propagators** — Keplerian elements and N-body integration
- **Star catalogs** — HYG Database (curated ~120k stars) or Gaia DR3

**Existing foundation:** [`github.com/LarsFlaeten/astro`](https://github.com/LarsFlaeten/astro) — SPICE integration, to be pulled in as a git submodule under `libs/astro`.

### Layer 2 — Scene Graph & Coordinate Management (`libs/universe`)

The bridge between physics truth and renderable scene.

- Origin recentering logic
- Hierarchical spatial index (octree or BVH)
- Level-of-detail orchestration — when to switch a body from a point mass to a rendered mesh
- Time-driven update loop driving the whole scene

### Layer 3 — Render Engine (`libs/render`)

All interesting rendering work is in atmosphere scattering, terrain LOD, and procedural textures — not in triangle submission — so full pipeline control matters.

**Chosen approach: Vulkan** with a thin bootstrap layer (vk-bootstrap + VMA). Raw enough to control the pipeline; not raw enough to spend months on swap chain boilerplate.

| Candidate | Decision |
|---|---|
| Raw Vulkan from scratch | Too slow to first visual |
| **Vulkan + vk-bootstrap + VMA** | ✅ Chosen — low level where it matters |
| Godot (Vulkan backend) | Too much abstraction |
| Bevy | Rust ecosystem too immature for this |

Key render subsystems:
- **Terrain LOD** — Proland-style geometry clipmaps ([proland.inria.fr](https://proland.inria.fr/)). Prior fork: [`github.com/LarsFlaeten/Proland_dev`](https://github.com/LarsFlaeten/Proland_dev). Needs porting to Vulkan.
- **Atmospheric scattering** — Bruneton's precomputed scattering model (gold standard; Vulkan version exists)
- **Star rendering** — point sprites / impostors, spectral color from catalog data
- **Procedural textures** — noise-based, physically parameterized by body type (rocky, icy, gas giant, etc.) for bodies without real texture data

### Layer 4 — Content & Procedural Generation

- Real texture data where available (Earth, Moon, Mars, etc.)
- Procedural fallback for asteroids, exoplanets, fictional bodies
- Physically parameterized by body classification

### Layer 5 — Spacecraft & Modular Objects

Deliberately last — designed once the world exists and real use cases are visible.

#### Physical object model
- Mass, centre of mass, inertia tensor per spacecraft
- Component system: engine pods, RCS clusters, landing legs, docking ports, sensors
- Per-spacecraft `.toml` sidecar manifest with Apeiron-specific properties (mass, CoM, named nodes for ports and engines, collision mesh)

#### Asset pipeline
OpenSCAD (parametric concept) → Blender (surface detail, PBR materials) → glTF 2.0 / `.glb` (runtime format, loaded via **fastgltf**)

#### Docking standard
All Apeiron spacecraft use **IDSS** (International Docking System Standard) ports, matching the real-world standard used by Crew Dragon, Starliner, and Gateway.
- Two-phase docking: soft capture (3-petal ring) → hard capture (mechanical hooks)
- Active / passive role negotiation per port
- Supports power, data, and air transfer post-mate

#### Assets

Three vehicles are planned, plus a hull-integrated drone cradle:

| Asset | File | Purpose |
|---|---|---|
| SpaceX Crew Dragon | `dragon2.glb` | Placeholder — orbital mechanics and renderer bringup |
| Custom multirole ship | `apeiron_ship.glb` | Long-term primary vessel (reentry, hover, landing, docking, elevator) |
| Utility drone / ROV | `apeiron_drone.glb` | Close-proximity inspection, manipulation, relay |
| Drone Cradle Interface | (part of ship glTF) | Hull-integrated stow, charge, and launch system for drone |

Parametric OpenSCAD concept models: `apeiron_ship.scad`, `apeiron_drone.scad`.

Key design decisions for the custom ship:
- **Lifting body**, no wings; 18 m × 11 m × 5.5 m default dimensions
- **No cockpit** — flush camera clusters, crew inside hull
- **Four articulating engine pods** — fold back for orbit, swing down for hover (toed out 12°)
- **Six RCS clusters** for full 6-DOF vacuum authority
- **Dorsal IDSS port** (passive, load-bearing) — space elevator and station docking
- **Nose IDSS port** (active) — ship-to-ship docking only
- **Belly** reserved for ceramic heat shield and landing legs; no ventral port

The drone is Astrobee-sized (32×32×20 cm), cold-gas N₂ RCS, ~8 m/s ΔV, LIDAR-guided
autonomous return to its DCI cradle. The DCI is fully flush with the hull when closed.

See `docs/design/spacecraft.md` for full design rationale, sidecar manifest format, and build-out order.

---

## Recommended Build-Out Order

1. **Coordinate & time system** — SPICE integrated, CLI tool that outputs body positions at arbitrary dates. Validate against JPL Horizons.
2. **Minimal Vulkan renderer** — triangle → textured sphere → star skybox from HYG catalog.
3. **Connect them** — render solar system bodies as discs/points at correct SPICE positions with correct relative motion.
4. **Terrain LOD** — single body (Moon is good: no atmosphere to worry about yet), Proland-style geometry.
5. **Atmosphere & lighting** — Bruneton scattering on Earth.
6. **Procedural content** — extend to other solar system bodies, then fictional/exo bodies.
7. **Spacecraft** — modular object system, first flyable vehicle.

---

## Tech Stack

| Concern | Choice |
|---|---|
| Language | C++23 |
| Build system | CMake (monorepo) |
| Dependency management | vcpkg (manifest mode, `vcpkg.json`) |
| Ephemeris | CSPICE (NASA/NAIF C library) |
| Math | GLM (render/geometry), Eigen if heavy linear algebra needed |
| Renderer | Vulkan via Vulkan-Hpp, vk-bootstrap, VMA |
| Windowing | GLFW or SDL3 |
| Atmosphere | Bruneton precomputed scattering (ported to Vulkan) |
| Star catalog | HYG Database |
| glTF loader | fastgltf |
| Testing | Catch2 |

---

## Repository Structure

```
apeiron/
├── CMakeLists.txt              # root — ties everything together
├── vcpkg.json                  # dependency manifest
├── cmake/                      # find modules, toolchain helpers
├── extern/                     # git submodules (CSPICE, etc.)
├── libs/
│   ├── astro/                  # submodule: LarsFlaeten/astro
│   ├── universe/               # coordinate mgmt, scene graph
│   └── render/                 # Vulkan renderer
├── apps/
│   └── apeiron/                # main executable
├── tests/                      # integration tests
├── docs/                       # extended documentation
└── ARCHITECTURE.md             # this file
```

**C++ namespace:** `apeiron::` — libraries expose `apeiron::astro`, `apeiron::universe`, `apeiron::render`.

---

## Reference Projects & Resources

- [NASA/NAIF SPICE](https://naif.jpl.nasa.gov/naif/spiceconcept.html) — ephemeris and coordinate frames
- [Proland](https://proland.inria.fr/) — terrain and atmosphere rendering research
- [Bruneton atmospheric scattering](https://ebruneton.github.io/precomputed_atmospheric_scattering/) — physically based atmosphere
- [HYG Star Database](https://github.com/astronexus/HYG-Database) — 120k+ stars with spectral data
- [JPL Horizons](https://ssd.jpl.nasa.gov/horizons/) — ground truth for ephemeris validation
- [Orbiter Space Flight Simulator](http://orbit.medphys.ucl.ac.uk/) — spiritual reference point
