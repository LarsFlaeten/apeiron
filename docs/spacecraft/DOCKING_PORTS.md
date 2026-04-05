# Docking Port Convention

## Overview

Docking ports are modelled as Blender **Empty** objects (axis display) placed at the
docking surface.  The simulator discovers them by scanning glTF node names at startup.

---

## Node Naming

```
docking_port_<role>_<label>
```

| Part | Values | Meaning |
|------|--------|---------|
| `role` | `active` | Probe side — initiates capture (e.g. Orion CM) |
| | `passive` | Drogue side — receives the probe (e.g. ISS ports) |
| `label` | any snake_case string | Human-readable display name shown in UI |

**Examples**
```
docking_port_active_orion
docking_port_passive_harmony_fwd
docking_port_passive_harmony_top
docking_port_passive_pma2
```

### Active / Passive matching

The simulator only offers a port as a docking target when the player spacecraft has at
least one port of the **opposite** role:

| Player has | Compatible target ports |
|------------|------------------------|
| `active`   | `passive` ports only   |
| `passive`  | `active` ports only    |

---

## Axis Convention (model space, metres)

```
        +Z  ("up" / roll reference)
         |
         |
  +X ----o----  (outward, away from docking surface)
        /
       /
     (docking surface flush with Empty origin)
```

| Axis | Direction |
|------|-----------|
| **+X** | Points **outward** along the approach corridor — away from the docking surface, toward the incoming spacecraft |
| **+Z** | Points **up**, defining the roll-zero reference for alignment |
| **+Y** | Right-hand completion of the frame (= Z × X) |

The docking MFD will use these axes to compute:
- **Lateral offset** — deviation from the +X approach line
- **Roll error** — angle between player +Z and port +Z
- **Approach rate** — component of relative velocity along port +X

---

## Blender Setup

1. In the target spacecraft, add an **Empty → Axes** at the docking surface.
2. Rename it `docking_port_passive_<label>` (or `active` for a probe side).
3. Orient it so:
   - Local **+X** points outward (the direction an approaching ship comes from).
   - Local **+Z** points up relative to the port's own reference frame.
4. The Empty's **origin must sit flush with the physical docking surface** — not
   recessed or offset outward.  The simulator uses this position as the aim point for
   the approach guidance squares (future feature).
5. Parent the Empty to the relevant structural node so it moves with the model.

---

## gltfpack

Use **both** `-cc` and `-kn` together:

```bash
gltfpack -i model.glb -o model_opt.glb -cc -kn
```

| Flag | Effect |
|------|--------|
| `-cc` | meshopt compression — reduces file size significantly |
| `-kn` | Keep Named nodes — prevents gltfpack from pruning mesh-less Empties |

Without `-kn`, all Empty nodes (no mesh, no mesh-bearing children) are silently removed
and the simulator will find zero docking ports.

---

## Runtime Behaviour

| Condition | Effect |
|-----------|--------|
| Distance to target > 5 km | Port sub-selection unavailable; `[PORT]` button hidden |
| Distance ≤ 5 km + compatible port exists | `[PORT]` button appears in Nav Console |
| Port selected | HUD target box shifts to port world position; distance readout is port-to-port |
| Distance exceeds 5 km again | Port selection resets to spacecraft CoM automatically |

The 5 km threshold is `kPortSelectKm` in `main.cpp`.

---

## Current Port Inventory

### Orion CM (`CM_SM06.glb`)

| Node | Role | Position (model space, m) | Notes |
|------|------|--------------------------|-------|
| `docking_port_active_orion` | active | ~(3.46, 0, 0) | Forward CM probe, +X forward |

### ISS (`ISS_dports_opt.glb`)

| Node | Role | Position (model space, m) | Notes |
|------|------|--------------------------|-------|
| `docking_port_passive_harmony_fwd` | passive | ~(-16.4, -5.6, 0) | Harmony nadir/fwd |
| `docking_port_passive_harmony_top` | passive | ~(-11.9, 0.18, 0) | Harmony zenith |
