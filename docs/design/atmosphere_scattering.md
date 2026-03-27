# Atmospheric Scattering — Design Notes

## Overview

Apeiron implements physically-based single-scattering atmospheric rendering,
based on Bruneton & Neyret (2008/2017).  The goal is accurate limb glow and
surface haze for rocky planets (Earth, Mars, Venus), viewed primarily from
orbital distance.

---

## Algorithm

### Why single scattering?

Multiple-scattering (Bruneton's full model) requires five iterated compute
passes and a 4-D scattering LUT.  For a space simulator viewed from orbit,
single scattering captures the dominant visual effects:

- **Limb glow** — sunlit halo visible at the planet edge
- **Day-sky colour** — the diffuse blue illumination on the lit hemisphere
- **Opposition surge** — brighter atmosphere when sun is directly behind the viewer

Multiple scattering adds subtler ambient sky brightness and inner-limb fill.
It is a planned upgrade (marked in the backlog).

### Precomputed transmittance LUT

The most expensive per-frame operation is computing how much light survives
a ray through the atmosphere.  We precompute this once at load time into a
**256×64 RGBA32F** texture:

```
T(r, μ) = exp(-∫ [β_R·ρ_R(h) + β_Mext·ρ_M(h)] ds)
```

Parameterisation (linear; Bruneton uses a non-linear mapping for better
precision near the horizon — a future upgrade):

| texture axis | maps to |
|---|---|
| `u` (x) | normalised altitude: `(r − Rg) / (Ra − Rg)` |
| `v` (y) | `(cos zenith + 1) / 2` |

The LUT is filled by a single Vulkan **compute shader** dispatch
(`atmosphere_transmittance.comp`, workgroup 16×16, dispatch 16×4).
High sample count (500) is used for quality since this runs once.

### Runtime integration

At render time the atmosphere shell is a **unit sphere** in local space
(scaled to `atmosphereRadius` km by the model matrix).  The fragment shader:

1. Constructs a view ray from the camera (in local space) through the fragment.
2. Finds the ray–atmosphere and ray–ground intersections analytically.
3. Integrates along 16 steps inside the atmosphere:
   - Accumulates optical depth along the view ray.
   - Computes `T(camera→P)` from accumulated depth.
   - Looks up `T(P→sun)` from the precomputed LUT.
   - Accumulates Rayleigh and Mie in-scattered light.
4. Applies Rayleigh (Cauchy) and Mie (Henyey-Greenstein) phase functions.
5. Scales by `lightIntensity = 1/d²` (AU) for inverse-square solar falloff.

### Blend equation

The fragment outputs `(L_inscatter, transmittance_luma)` and uses a
non-standard Vulkan blend:

```
src factor = One
dst factor = SrcAlpha   (= transmittance_luma from fragment alpha output)
──────────────────────────────────────────────────────────────────
result.rgb = L_inscatter  +  background.rgb × transmittance_luma
```

This composites the scattered light on top of planet surface or space,
with the correct transmittance dimming of the background.

---

## Coordinate spaces

Everything is computed in **local space** where the atmosphere sphere = radius 1.

| quantity | local-space formula | example (Earth) |
|---|---|---|
| ground sphere radius | `Rg = Rg_km / Ra_km` | `6360/6460 ≈ 0.9845` |
| scattering coefficient | `β_local = β_km · Ra_km` | `β_R_blue = 33.1e-3 · 6460 ≈ 213.8` |
| scale height | `H_local = H_km / Ra_km` | `H_R = 8/6460 ≈ 1.24e-3` |

The camera and sun direction are transformed to local space on the CPU:

```cpp
glm::mat4 invModel   = glm::inverse(atmosphereModel);
glm::vec3 camLocal   = glm::vec3(invModel * glm::vec4(camera.position(), 1.0f));
glm::vec3 sunLocal   = glm::normalize(glm::vec3(invModel * glm::vec4(sunDir, 0.0f)));
```

This works correctly because `renderPos ≈ (0,0,0)` after scene recentering.

---

## Physical parameters

Earth (from Bruneton 2017, Table 1):

| parameter | value | unit |
|---|---|---|
| `Rg` | 6360 | km |
| `Ra` | 6460 | km (100 km thick) |
| `β_R` | (5.802, 13.558, 33.10) × 10⁻³ | km⁻¹ |
| `H_R` | 8.0 | km |
| `β_M scattering` | 3.996 × 10⁻³ | km⁻¹ |
| `β_M extinction` | 4.440 × 10⁻³ | km⁻¹ |
| `H_M` | 1.2 | km |
| `g` (Mie asymmetry) | 0.76 | — |

These are specified in `data/scenarios/scenario.toml` under the
`atmosphere` sub-table of each `[[bodies]]` entry.

---

## Pipeline architecture

```
AtmospherePrecompute
  ├── compute shader: atmosphere_transmittance.comp
  └── owns: transmittance LUT image (RGBA32F 256×64)

AtmospherePipeline
  ├── vertex shader:   atmosphere.vert
  ├── fragment shader: atmosphere.frag
  ├── descriptor layout: binding 0 = transmittance LUT (sampler2D)
  ├── push constants: 128 bytes (mat4 mvp + 4×vec4 params)
  └── blend: src=One, dst=SrcAlpha

Renderer::drawAtmosphere()
  └── called in HDR pass, after body draw, before ring draw
      (depth test on, depth write off, back-face culled)
```

---

## Rendering order (within HDR pass)

1. Stars (no depth write)
2. Planet body (opaque, depth write)
3. **Atmosphere shell** (transparent, depth test, no depth write)  ← NEW
4. Ring system (transparent, depth test, depth write)

The atmosphere shell is drawn after the planet so the planet's depth values
are already in the buffer; atmosphere fragments behind the planet surface are
correctly discarded.

---

## LUT inspector

An "Atmosphere LUTs" ImGui window shows the precomputed transmittance LUT
as a 2-D image:

- **Horizontal axis (u)**: altitude (left = ground, right = top of atmosphere)
- **Vertical axis (v)**: cos(zenith angle) (top = looking down, bottom = looking up)
- **Colour**: transmittance per wavelength (red ≈ 680nm, green ≈ 550nm, blue ≈ 440nm)

The LUT should appear dark at low altitude + small zenith cosine (ray skims
along the ground through a long atmospheric path) and bright at high altitude
or large zenith cosine (short path to space).

---

## Known limitations / backlog

| item | notes |
|---|---|
| Single scattering only | Multiple scattering would add ambient sky fill and brighter inner-limb glow |
| Linear LUT parameterisation | Bruneton uses a non-linear mapping for better horizon precision |
| No ground irradiance | Reflected sky light on the surface not modelled |
| No in-atmosphere shadow (self-shadowing) | Sun below horizon is a sharp cutoff |
| No atmospheric refraction | Relevant for grazing-angle sunrise/sunset views |
| Gas giants | Layered cloud-deck atmospheres need a different model |
| Venus | Extremely thick CO₂; will need higher step count |
| Camera inside atmosphere | Currently skipped (sizeScale > 1 guard); needs interior rendering path |
