# Spacecraft & Drone Asset Design Notes

## Overview

Apeiron's spacecraft layer sits above the physics and rendering foundations.
Any spacecraft or autonomous vehicle is described by a TOML sidecar manifest,
loaded as a glTF asset, and simulated with correct dynamics.

Assets in scope:
1. **SpaceX Crew Dragon** — placeholder vessel for early bringup
2. **Custom Apeiron multirole ship** — long-term primary crewed vessel
3. **Utility drone / ROV** — close-proximity operations vehicle
4. **Drone Cradle Interface (DCI)** — hull-integrated stow, power, and launch system

---

## Asset pipeline (all vessels)

```
OpenSCAD            parametric concept model — iterate proportions cheaply
   ↓  (export STL)
Blender             remesh, surface detail (panel lines, thermal tiles,
                    sensor clusters, camera housings), PBR material zones
   ↓  (export glTF 2.0)
.glb                runtime format — loaded in Apeiron via fastgltf
   +
.toml sidecar       Apeiron-specific properties (see below)
```

Named nodes (`port_forward`, `engine_pod_1`, `dci_01`, …) are bones or empty
objects in the glTF scene graph; Apeiron resolves their world transforms each frame.

---

## Docking standard — IDSS

All Apeiron crewed and cargo vessels use the **International Docking System
Standard** (IDSS), the real-world standard used by Crew Dragon, Starliner,
and the Lunar Gateway. The drone DCI is a separate smaller proprietary
interface — IDSS is too large and complex for a small drone cradle.

### Mechanism
1. **Soft capture** — 3-petal ring engages; small misalignment is corrected.
2. **Hard capture** — 12 hooks lock; structural loads can now be transferred.
3. **Post-mate** — power, data, and air transfer through the interface ring.

### Port roles
| Role | Description |
|---|---|
| Active | Initiates capture; carries the 3-petal ring mechanism |
| Passive | Receives the ring; carries the hook ring |

A port may be manufactured for either or both roles; role is negotiated
in software per mission.

---

## 1. Placeholder vessel — SpaceX Crew Dragon

For renderer and orbital-mechanics bringup before the custom ship is ready.

| Property | Value |
|---|---|
| Model source | NASA 3D Resources (OBJ, free) |
| Runtime format | glTF 2.0 (convert via Blender or obj2gltf) |
| Mass | 12 519 kg (capsule + trunk) |
| Diameter | 3.66 m |
| Height | 8.1 m (with trunk) |
| Docking port | Nose — IDSS active |

**Limitations:** capsule-only; no aerodynamic lift, no hover, no landing legs.
Suitable for orbital mechanics, docking, and renderer bringup. Not suitable for
atmospheric flight or surface operations.

### Sidecar manifest

```toml
[spacecraft.dragon2]
model             = "dragon2.glb"
mass_kg           = 12519.0
center_of_mass    = [0.0, 0.0, 1.4]   # metres, model-local

[[spacecraft.dragon2.docking_ports]]
node = "port_forward"
type = "IDSS_active"

[[spacecraft.dragon2.engines]]
node   = "engine_draco_1"
type   = "draco"
thrust = 400.0   # N, vacuum

collision_mesh = "dragon2_collision.glb"
```

---

## 2. Custom Apeiron multirole ship

Full mission coverage: orbital transit, reentry, hover, surface landing,
ship-to-ship docking, and space elevator attachment.

Parametric OpenSCAD concept model: **`apeiron_ship.scad`**
Default dimensions: 18 m long × 11 m wide × 5.5 m tall (all tunable).

Inspiration: the Deltaglider concept from Orbiter — departing significantly.
No wings, no fighter-jet canopy; shape is driven by physics and function.

### Hull

- **No cockpit or canopy.** Crew is inside the pressure hull; navigation via
  flush-mounted external camera clusters feeding interior displays. Shape is
  not constrained by human eyeline requirements.
- **Lifting body.** Generates body lift on reentry — no wings needed.
  Profile: ovoid/teardrop, wider than tall, flattened belly.
- **Integrated ceramic heat shield.** The belly is flattened and the material
  zone is coloured as ceramic tile. Not a bolt-on plate — the hull shape
  itself provides the blunt-body reentry geometry.

### Propulsion

- **Four articulating engine pods** at the hull corners.
  - Orbital/reentry: folded back against hull, low drag.
  - Hover/landing: swing down ~90°, toed out ~12° for hover stability.
  - Attitude: gimballed for pitch/yaw/roll authority.
- **Six RCS clusters** distributed around hull for full 6-DOF authority
  (translate X/Y/Z, pitch, yaw, roll) in vacuum.
- **Four deployable landing legs** — wide stance, passive suspension,
  suitable for uneven terrain.
- Propulsion is non-realistic (unlimited fuel) initially; realistic
  propellant mass and specific impulse to be added later.

### Cameras

| Cluster | Location | Primary use |
|---|---|---|
| Forward | Nose face | Navigation, nose docking approach |
| Aft | Tail | Departure, proximity |
| Belly | Lower hull | Landing |
| Dorsal | Roof | Station / space elevator docking approach |
| Side (×2) | Port and starboard flanks | Situational awareness |

### Docking ports

| Port | Location | Role | Notes |
|---|---|---|---|
| Primary | Dorsal (roof) | IDSS passive | Load-bearing — elevator attachment and station docking. Ship approaches from below, hangs under car/station. Takes tension loads. |
| Secondary | Nose | IDSS active | Ship-to-ship docking only. Cannot take axial structural loads; not used for elevator. |
| — | Ventral (belly) | — | Reserved for heat shield and landing legs. No port. |

#### Space elevator compatibility

The dorsal port is the elevator attachment point. Ship approaches the elevator
car from below; dorsal port hard-captures to the car's passive port. Under
elevator acceleration the ship hangs in tension — this port must carry full
ship mass. The nose port is not used for elevator attachment.

---

## 3. Utility drone / ROV

Close-proximity operations around spacecraft: inspection, manipulation,
external repairs, relay.

Parametric OpenSCAD concept model: **`apeiron_drone.scad`**
Real-world reference: NASA Astrobee (ISS free-flyer robot).

### Body

32 × 32 × 20 cm rounded rectangular box.

### Propulsion — cold gas RCS (N₂)

| Component | Count | Function |
|---|---|---|
| Corner pods (±X nozzle pairs) | 4 | X translation + yaw |
| Top-mounted nozzles (±Y) | 2 | Y translation + pitch |
| Reaction wheels (orthogonal) | 3 | Fine attitude hold + roll |

No combustion — safe near airlocks, sensitive surfaces, and crew.

| Propellant budget | Value |
|---|---|
| N₂ mass | ~0.5 kg |
| Delta-V | ~8 m/s (close-proximity ops) |

### Sensors

| Sensor | Location | Purpose |
|---|---|---|
| Main navigation camera | Forward face, centre | Primary navigation |
| Stereo auxiliary cameras (×2) | Forward face | Depth perception |
| LIDAR proximity ring | Forward face perimeter | Obstacle avoidance, docking alignment |

### Power

- Internal ~200 Wh battery
- Recharged via DCI contact plate when stowed

### Docking face (aft)

- 2×3 gold electrical contact pin array
- Two mechanical latch receivers (port and starboard)
- Drone backs autonomously into DCI cradle using LIDAR ring + aft camera on mothership

### Sidecar manifest

```toml
[spacecraft.drone_01]
model         = "apeiron_drone.glb"
mass_kg       = 8.5
propellant_kg = 0.5
propellant    = "cold_gas_n2"
delta_v_ms    = 8.0
battery_wh    = 200
camera_nodes  = ["cam_main", "cam_aux_l", "cam_aux_r"]
lidar_node    = "lidar_ring"
dci_compat    = true
```

---

## 4. Drone Cradle Interface (DCI)

Hull-integrated system for stowing, powering, and launching the drone.

### Mechanical

- Fully recessed into mothership hull — zero external protrusion when closed.
- Hatch closes flush with hull skin when drone is deployed; swings
  outward/downward before launch.
- Tapered alignment guide rails guide drone into seat on return.
- Spring ejector provides ~0.3 m/s separation velocity on launch.
- Two spring-loaded latch pins engage drone latch receivers; retract via
  solenoid on release command.
- Drone docks autonomously using its LIDAR ring + aft camera on mothership.

### Electrical

- Contact plate on cradle aft wall mates with drone contact pins.
- Supplies: 28 V power, SpaceWire data bus.
- Charges battery and uploads mission data while stowed.

### Integration

- Multiple DCIs can be placed on different hull faces (belly, dorsal, flanks).
- glTF node naming convention: `dci_01`, `dci_02`, drone node `drone_01`, etc.

### Sidecar manifest

```toml
[dci.dci_01]
node          = "dci_01"
position      = "belly_port"
voltage_v     = 28
bus           = "spacewire"
drone_default = "drone_01"
```

---

## Build-out order

1. **Dragon glTF** — load and render; validate node resolution and PBR materials.
2. **Orbital mechanics** — propagator, maneuver planning; Dragon as test vehicle.
3. **Docking** — IDSS soft/hard capture; Dragon docks to a station stand-in.
4. **Custom ship — geometry** — OpenSCAD → Blender → glTF import.
5. **Custom ship — articulation** — engine pod swing, landing leg deploy.
6. **Hover and landing** — thruster model, terrain collision, leg contact.
7. **Reentry** — aerodynamic lift/drag on lifting body; heat shield thermal state.
8. **Drone** — load, render, autonomous proximity flight around ship.
9. **DCI** — cradle hatch animation, latch/release, power handshake.
10. **Space elevator** — dorsal port attachment; cable tension dynamics.
