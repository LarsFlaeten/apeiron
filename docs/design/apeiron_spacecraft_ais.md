# Apeiron: Spacecraft Situational Awareness System (SCAS)

## Concept Overview

A simulated broadcast-based situational awareness system for all active entities in the Apeiron simulation. Inspired by maritime AIS and aviation ADS-B, every vessel, space station, and relevant object periodically broadcasts a beacon packet containing its orbital state and metadata. Receivers aggregate these beacons to maintain a live picture of the local and system-wide space environment.

This system underpins MFD navigation pages, docking approach, autopilot targeting, and any future multi-vessel interaction.

---

## Real-World Basis

| Analog | Domain | Mechanism |
|---|---|---|
| AIS | Maritime | Self-reported VHF broadcast, shore relay |
| ADS-B | Aviation | GPS-derived position broadcast, ground/air relay |
| CCSDS OMM/OEM | Space (existing) | Standardised orbit element/ephemeris exchange |
| ADS for Space | Space (proposed) | Direct application of ADS-B concept to spacecraft |

The closest existing real-world equivalent is **Automatic Dependent Surveillance (ADS)** adapted to orbital mechanics. SSA programmes (US SSN, ESA) track objects from the ground; SCAS is the self-reported complement.

---

## Architecture: Hybrid Broadcast + Relay

### Two-tier model

```
[Vessel A] ──broadcast──▶ [Local radio bus, range-limited]
[Vessel B] ──broadcast──▶ [Local radio bus, range-limited]
                               │
                        [Space Station]  ← also a broadcaster
                        - Aggregates received beacons
                        - Maintains local registry
                        - Rebroadcasts registry at lower rate (system-wide)
                               │
                        [Orion MFD] ← subscribes to both tiers
```

**Tier 1 — Direct beacon (short range, high fidelity)**
- All entities broadcast on a fixed interval (e.g. every sim-second or configurable per entity class)
- Received directly by any entity within radio range
- Primary source for proximity ops, rendezvous, docking

**Tier 2 — Relay registry (system-wide, lower update rate)**
- Space stations and relay satellites aggregate Tier 1 beacons
- Re-broadcast a registry digest covering all known entities
- Used for system-wide situational awareness on the nav MFD

---

## Beacon Packet Format

### Base beacon (all entities)

```cpp
struct ScasBeacon {
    // Identity
    EntityGUID      id;               // Simulation-wide unique ID
    std::string     name;             // Human-readable callsign/name
    EntityClass     entity_class;     // CREWED_VESSEL, STATION, CARGO, DEBRIS, PROBE

    // Timing
    double          epoch_tai;        // Broadcast epoch, TAI seconds

    // Orbital state (J2000 / ICRF)
    Vec3d           position_m;       // Position vector, metres
    Vec3d           velocity_mps;     // Velocity vector, m/s

    // Attitude
    Quaterniond     orientation;      // Body frame orientation in J2000
    Vec3d           angular_velocity; // rad/s, body frame

    // Status
    float           fuel_fraction;    // 0.0–1.0, optional / spoofable
    bool            extended_data;    // Flag: extended packet follows
};
```

### Extended beacon — docking manifest (stations and vessels with ports)

```cpp
struct DockingPort {
    uint8_t         port_id;
    std::string     port_name;        // e.g. "PMA-2", "Nadir Port A"
    Vec3d           position_body;    // Port position in body frame
    Quaterniond     approach_frame;   // Docking axis orientation
    DockingStatus   status;           // FREE, OCCUPIED, RESERVED, DAMAGED
    EntityGUID      occupant_id;      // Valid if OCCUPIED
};

struct ScasBeaconExtended : ScasBeacon {
    std::vector<DockingPort> docking_ports;
    // Future: comms frequencies, approach corridors, keep-out zones
};
```

---

## Simulation Implementation Notes

### ECS / component model

- Add a `ScasTransmitterComponent` to any entity that broadcasts
  - Configurable: broadcast interval, power level (affects range), beacon type (base / extended)
- Add a `ScasReceiverComponent` to any entity that consumes beacons
  - Maintains a `std::map<EntityGUID, ScasBeacon>` of last-received beacons with timestamps
  - Entries expire after a configurable timeout (entity lost contact)

### Radio bus

- A simulated `ScasRadioBus` system ticks each frame
- For each transmitter, computes which receivers are within range (simple sphere check, or later: occultation by bodies)
- Delivers beacon copy to each in-range receiver's cache
- **Emergent behaviour**: losing line-of-sight behind a planet naturally drops the beacon

### MFD integration

The nav/radar MFD page queries the Orion's `ScasReceiverComponent` cache:
- **Proximity view**: direct Tier 1 beacons, full fidelity, relative position/velocity
- **System map view**: Tier 2 relay registry, all known entities, lower update rate
- Stale beacons rendered differently (greyed out, last-known position shown with uncertainty cone)

---

## Update Rates (Suggested)

| Entity class | Broadcast interval | Rationale |
|---|---|---|
| Crewed vessel (active) | 1 s | Proximity ops safety |
| Crewed vessel (coasting) | 10 s | Low activity |
| Space station | 5 s | Docking port status changes |
| Cargo / probe | 30 s | Low priority |
| Relay registry digest | 60 s | System-wide, low bandwidth |

---

## Future Extensions

- **Approach corridor broadcast**: station broadcasts a structured approach path per port (waypoints + speed gates), consumed by autopilot
- **Traffic advisories**: TCAS-style conflict detection when two vessels are on converging orbits
- **Comms frequencies**: entities advertise their active radio channels
- **Transponder modes**: active (full beacon), passive (position only), silent (military/emergency)
- **Ground stations**: planetary surface installations act as Tier 2 relay nodes with planetary coverage

---

## Open Design Questions

1. **Reference frame for beacon position** — J2000 barycentric, or solar system barycentric (ICRF)? ICRF is more correct for SPICE integration.
2. **Beacon authentication** — in a future multiplayer or adversarial scenario, should beacons be spoofable? Probably yes, as a gameplay mechanic.
3. **Relay station discovery** — how does a vessel know which stations are acting as relays? Options: well-known frequency, Tier 1 beacon flag, or hardcoded by scenario.
4. **Bandwidth simulation** — model actual bit rate limits, or treat radio as lossless within range?
