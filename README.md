# Apeiron

*ἄπειρον* — Anaximander's word for the boundless, unlimited substrate of the universe.

Apeiron is a physically-correct space simulator built from first principles: real ephemeris, real orbital mechanics, and physically-based rendering. The long-term north star is a **hardware-in-the-loop spacecraft simulator** where a LEON3 processor emulation (the chip family used in real ESA missions) runs actual GNC software against a live simulated solar system.

---

## Screenshots

![Earth from orbit](docs/screenshots/Earth.png)
*Earth with Bruneton single-scattering atmosphere — blue limb glow and terminator haze*

![Mars from orbit](docs/screenshots/Mars.png)
*Mars with dust-reddened atmosphere — reddish-orange limb characteristic of iron-oxide aerosols*

![Saturn rings](docs/screenshots/Saturn.png)
*Saturn's ring system with Lommel-Seeliger opposition surge*

---

## What works today

### Solar system bodies
- All eight planets + Moon, Phobos, Deimos rendered at SPICE-correct positions
- Positions and orientations from **NASA/NAIF SPICE kernels** (DE440s + planetary PCK)
- Physically correct **oblate spheroids** (tri-axial ellipsoid radii from PCK)
- High-resolution real texture maps (8K for Earth, Mars, Moon, Jupiter, Saturn, …)
- Normal mapping and specular mapping (Earth)
- **Displacement / tessellation LOD** — MOLA DEM for Mars, LOLA DEM for Moon; height displaced on the GPU via tessellation shaders with view-distance LOD
- **Irregular mesh bodies** — Phobos and Deimos rendered from real shape models (OBJ)
- Cloud layer (Earth)

### Atmospheric scattering
Bruneton & Neyret single-scattering model, precomputed at load time:

- **Transmittance LUT** (256×64 RGBA32F) filled by a Vulkan compute shader; parameterised by altitude and cos(zenith)
- Per-body atmosphere parameters (Rayleigh β per wavelength, Mie β, scale heights, Henyey-Greenstein g)
- **Earth** — Bruneton 2017 Table 1 parameters; characteristic blue limb glow and terminator haze
- **Mars** — thin CO₂ atmosphere with dust-reddened scattering; reddish/orange limb glow
- Physically correct single-scattering integration (16 steps) with LUT-based sun transmittance lookup
- Blend equation: `result = L_inscatter + background × transmittance` (non-standard Vulkan src=One, dst=SrcAlpha)
- ImGui **Atmosphere LUT inspector** — real-time view of the transmittance texture per planet with altitude/zenith tooltip

### Ring system
- Saturn's ring system rendered from a real ring texture (alpha channel encodes opacity profile)
- **Lommel-Seeliger scattering** model — physically motivated single-particle BRDF for regolith surfaces
- Correct **opposition surge** — brightening when the sun is directly behind the observer

### Rendering pipeline
- **Vulkan** renderer (Vulkan-Hpp, vk-bootstrap, Vulkan Memory Allocator)
- HDR render pass with **bloom post-process** (threshold + Gaussian blur, two-pass)
- **Tone mapping** (Reinhard) in a final fullscreen pass
- Exposure control
- **Star field** — HYG Database v4.2 (~120 000 stars), spectral colour from B−V index, rendered as calibrated point sprites
- Depth-correct rendering order: stars → planets → atmosphere shells → rings
- Depth test on / depth write off for transparent layers (atmosphere, rings)

### Universe & camera
- **64-bit double precision** positions throughout; camera-relative float only at the GPU boundary
- **Origin recentering** each frame — floating origin stays near the camera, preventing float precision loss at large distances
- SPICE-driven simulation time with adjustable speed (seconds/second)
- Click-to-focus body selection; orbital camera (azimuth/elevation/distance)
- Distance display in km and AU

### Developer tools
- **ImGui** dev view: FPS, exposure, bloom controls, simulation speed, camera info, body distance list
- Atmosphere LUT inspector with per-planet LUT display
- Displacement scale slider for DEM exaggeration

---

## Architecture overview

```
apeiron/
├── libs/
│   ├── astro/       # SPICE wrapper, time system, coordinate frames
│   ├── universe/    # Scene graph, body catalogue, SPICE ephemeris queries
│   └── render/      # Vulkan renderer — pipelines, meshes, atmosphere, rings
├── apps/
│   └── apeiron/     # Main executable + GLSL shaders
├── data/
│   ├── scenarios/   # scenario.toml — bodies, kernels, atmosphere params
│   ├── kernels/     # SPICE kernels (LSK, PCK, SPK)
│   ├── textures/    # Planet texture maps
│   └── meshes/      # Phobos / Deimos shape models
└── docs/design/     # Design notes (atmosphere scattering, etc.)
```

All physics is **C++ namespaced** under `apeiron::astro`, `apeiron::universe`, `apeiron::render`.

---

## Tech stack

| Concern | Choice |
|---|---|
| Language | C++23 |
| Build | CMake (monorepo) + vcpkg |
| Ephemeris | NASA/NAIF CSPICE |
| Math | GLM |
| Renderer | Vulkan (Vulkan-Hpp, vk-bootstrap, VMA) |
| Windowing | GLFW |
| UI | Dear ImGui |
| Atmosphere | Bruneton precomputed scattering (Vulkan compute) |
| Star catalog | HYG Database v4.2 |
| Shape models | TinyObjLoader |
| Testing | Catch2 |

---

## Building

### Prerequisites
- Vulkan SDK (glslc must be on PATH)
- CMake ≥ 3.25
- vcpkg (set `VCPKG_ROOT`)

```bash
cmake --preset default        # configures + fetches vcpkg dependencies
cmake --build build
./build/bin/apeiron
```

Shaders are compiled from GLSL to SPIR-V at build time by `glslc`; paths are baked in at compile time.

---

## Planned / backlog

| Item | Notes |
|---|---|
| Multiple scattering | Adds ambient sky fill and inner-limb glow; requires 5-pass Bruneton precompute |
| Non-linear LUT parameterisation | Better horizon precision near grazing angles |
| Camera inside atmosphere | Interior sky rendering for surface/low-orbit views |
| Terrain geometry clipmaps | Proland-style for continuous LOD from orbit to surface |
| Procedural textures | Noise-based fallback for bodies without real texture data |
| Gas giant atmospheres | Layered cloud-deck model; different from rocky-planet Bruneton |
| Venus atmosphere | Extremely thick CO₂; needs higher integration step count |
| Galilean moons | Requires large SPK satellite kernel (~1.1 GB) |
| Spacecraft module | Modular physical object system |
| LEON3 HIL integration | OBC emulator running real GNC software against live sim |

---

## References

- [NASA/NAIF SPICE](https://naif.jpl.nasa.gov/) — ephemeris and coordinate frames
- [Bruneton & Neyret 2008 / 2017](https://ebruneton.github.io/precomputed_atmospheric_scattering/) — precomputed atmospheric scattering
- [HYG Star Database](https://github.com/astronexus/HYG-Database) — 120 k stars with spectral data
- [Lommel-Seeliger scattering](https://en.wikipedia.org/wiki/Lommel%E2%80%93Seeliger_law) — regolith BRDF for rings and airless bodies
- [Proland](https://proland.inria.fr/) — terrain LOD and atmosphere research (future terrain reference)
