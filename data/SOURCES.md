# Asset Sources

# Texture Sources

> Paths are relative to `data/textures/`.

## Mercury

| File | Description | Source | License |
|------|-------------|--------|---------|
| `mercury/8k_mercury.jpg` | Diffuse colour map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |

## Venus

| File | Description | Source | License |
|------|-------------|--------|---------|
| `venus/8k_venus_surface.jpg` | Surface map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `venus/4k_venus_atmosphere.jpg` | Atmosphere/cloud layer, 4096×2048 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |

## Earth

| File | Description | Source | License |
|------|-------------|--------|---------|
| `earth/8k_earth_daymap.jpg` | Diffuse colour map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `earth/8k_earth_normal_map.jpg` | Normal map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `earth/8k_earth_specular_map.jpg` | Specular (ocean) mask, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `earth/8k_earth_clouds.jpg` | Cloud layer, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `earth/8k_earth_normal_map.tif` | Normal map source (TIFF), unused at runtime | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `earth/8k_earth_specular_map.tif` | Specular map source (TIFF), unused at runtime | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |

## Moon

| File | Description | Source | License |
|------|-------------|--------|---------|
| `moon/8k_moon.jpg` | Diffuse colour map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `moon/lola_8k_height.png` | Displacement/height map, 8192×4096, 16-bit grayscale, units = metres. Resampled from the 118 m/px LRO LOLA DEM. | NASA / USGS Astrogeology — [LRO LOLA Global DEM 118m](https://astrogeology.usgs.gov/search/map/Moon/LRO/LOLA/Lunar_LRO_LOLA_Global_LDEM_118m_Mar2014) | Public domain (US Government work) |

## Mars

| File | Description | Source | License |
|------|-------------|--------|---------|
| `mars/8k_mars.jpg` | Diffuse colour map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `mars/MarsNormal.png` | Normal map, 8192×4096 (extracted from `2k4k8kMarsNormal.zip`) | [Celestia Motherlode — Mars bump maps](http://www.celestiamotherlode.net/catalog/marsbump.html) | **TODO: confirm licence** |
| `mars/2k4k8kMarsNormal.zip` | Normal map archive (2K/4K/8K versions), source for the PNG above | [Celestia Motherlode — Mars bump maps](http://www.celestiamotherlode.net/catalog/marsbump.html) | **TODO: confirm licence** |
| `mars/Mars_MGS_MOLA_8k_height.png` | Displacement/height map, 8192×4096, 16-bit grayscale, units = metres above areoid. Resampled from the 463 m/px DEM. | NASA / USGS Astrogeology — [Mars MGS MOLA DEM](https://astrogeology.usgs.gov/search/map/Mars/GlobalSurveyor/MOLA/Mars_MGS_MOLA_DEM_mosaic_global_463m) | Public domain (US Government work) |
| `mars/Mars_MGS_MOLA_DEM_mosaic_global_463m.tif` | Original MOLA DEM at 463 m/px (source for the height PNG above), not loaded at runtime | NASA / USGS Astrogeology | Public domain (US Government work) |

## Jupiter

| File | Description | Source | License |
|------|-------------|--------|---------|
| `jupiter/8k_jupiter.jpg` | Diffuse colour map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |

## Saturn

| File | Description | Source | License |
|------|-------------|--------|---------|
| `saturn/8k_saturn.jpg` | Diffuse colour map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |
| `saturn/8k_saturn_ring_alpha.png` | Ring transparency/density map, 8192×4096 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |

## Uranus

| File | Description | Source | License |
|------|-------------|--------|---------|
| `uranus/2k_uranus.jpg` | Diffuse colour map, 2048×1024 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |

## Neptune

| File | Description | Source | License |
|------|-------------|--------|---------|
| `neptune/2k_neptune.jpg` | Diffuse colour map, 2048×1024 | [Solar System Scope](https://www.solarsystemscope.com/textures/) | CC BY 4.0 |

# Mesh Sources

| File | Description | Source | License |
|------|-------------|--------|---------|
| `meshes/phobos/phobos_gaskell_64.obj` | Phobos shape model, Q=64 ICQ (49 152 triangles, smooth normals). Converted from PDS4 ICQ format (`phobos_quad64q.tab`). | [Gaskell Phobos Shape Model — PDS SBN](https://sbn.psi.edu/pds/resource/phobosshape.html) | Public domain (NASA PDS) |
| `meshes/deimos/deimos_g_167m_spc_obj_0000n00000_v002.obj` | Deimos shape model, 167 m/px SPC OBJ (~23 k triangles). Credit: Ernst et al. 2023 doi:10.1186/s40623-023-01814-7 | [SBMT — JHU APL Shared Files](https://sbmt.jhuapl.edu/shared-files/) | Public domain (NASA-funded) |

## Notes

- Solar System Scope CC BY 4.0 requires attribution in any distributed product.
- MOLA data is a product of NASA's Mars Global Surveyor mission (MGS, 1996–2006),
  instrument PI: David E. Smith (GSFC). No licence restrictions on US Government works.
- The `.aux.xml` sidecar file next to the MOLA PNG is a GDAL auxiliary metadata file
  generated during the reprojection/resample step; it is not a texture asset.
