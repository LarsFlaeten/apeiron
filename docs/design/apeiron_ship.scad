// =============================================================================
// Apeiron Multirole Spacecraft - Parametric Model
// =============================================================================
// A physics-motivated multirole spacecraft for the Apeiron space simulator.
//
// Design principles:
//   - No cockpit/canopy - crew inside, navigation via cameras
//   - Lifting body hull - body lift on reentry, no wings
//   - Blunt ceramic belly - reentry heat shield, belly-first attitude
//   - Four articulating engine pods - hover, VTOL, orbital maneuvering
//   - Dorsal IDSS port - primary structural port for space elevator + stations
//   - Nose IDSS port  - secondary active port for ship-to-ship docking
//   - Four deployable landing legs - wide stance
//   - RCS thruster clusters distributed around hull
//
// Usage:
//   Adjust parameters in the PARAMETERS section below.
//   Render sections individually by toggling the SHOW_* flags.
//
// Requires OpenSCAD 2021.01 or later.
// =============================================================================

// -----------------------------------------------------------------------------
// PARAMETERS
// -----------------------------------------------------------------------------

// Overall hull dimensions (metres scale — 1 unit = 1 metre)
hull_length     = 18.0;   // nose to tail
hull_width      = 11.0;   // widest point (beam)
hull_height     =  5.5;   // total height

// Hull shape controls
hull_nose_frac  = 0.28;   // fraction of length that is the nose taper
hull_tail_frac  = 0.22;   // fraction of length that is the tail taper
belly_flatten   = 0.18;   // how much the belly is flattened (0=round, 0.3=very flat)

// Heat shield — how far up from belly_z the dark ceramic colour extends
belly_band      = 0.55;   // fraction of hull half-height covered by ceramic colour

// Engine pods
pod_count       = 4;      // must be 4 (one per corner)
pod_length      = 3.2;    // length of each engine pod
pod_width       = 1.1;    // width of each engine pod
pod_height      = 1.0;    // height of each engine pod
pod_nozzle_r    = 0.38;   // nozzle bell radius
pod_offset_x    = 3.8;    // longitudinal offset from centre (fore/aft)
pod_offset_y    = 4.2;    // lateral offset from centre (port/starboard)
pod_cant_angle  = 12;     // toe-out angle of pods (degrees) - aids stability

// Landing legs
leg_count       = 4;
leg_length      = 3.8;    // deployed leg length
leg_width       = 0.18;   // leg strut width
leg_spread      = 5.8;    // half-span of deployed legs at foot
leg_height      = 2.2;    // height of foot above ground plane

// Docking ports
dorsal_port_r   = 0.42;   // IDSS port radius (~840mm diameter per spec)
dorsal_port_h   = 0.35;   // port collar height
dorsal_port_x   = 1.5;    // longitudinal position (positive = forward of centre)

nose_port_r     = 0.42;   // nose IDSS port radius
nose_port_h     = 0.40;   // nose port protrusion

// RCS thruster clusters
rcs_box_size    = 0.28;   // size of RCS thruster block
rcs_nozzle_r    = 0.06;   // individual RCS nozzle radius
rcs_nozzle_l    = 0.10;   // nozzle protrusion

// Camera clusters
cam_r           = 0.08;   // camera dome radius
cam_protrude    = 0.04;   // how much camera protrudes from hull

// Detail level
$fn = 48;                 // cylinder resolution (lower for fast preview, 96+ for render)

// Show/hide sections for development
SHOW_HULL        = true;
SHOW_BELLY       = true;   // dark ceramic zone on hull belly (no extra geometry)
SHOW_ENGINE_PODS = true;
SHOW_LANDING_LEGS= true;
SHOW_DORSAL_PORT = true;
SHOW_NOSE_PORT   = true;
SHOW_RCS         = true;
SHOW_CAMERAS     = true;

// Colour scheme (for preview — not used in STL export)
COL_HULL        = [0.55, 0.57, 0.60, 1.0];  // cool grey alloy
COL_BELLY       = [0.20, 0.18, 0.16, 1.0];  // dark ceramic
COL_POD         = [0.45, 0.47, 0.50, 1.0];  // slightly darker pod
COL_NOZZLE      = [0.30, 0.28, 0.26, 1.0];  // dark exhaust metal
COL_PORT        = [0.70, 0.72, 0.75, 1.0];  // light docking collar
COL_RCS         = [0.60, 0.60, 0.58, 1.0];
COL_LEG         = [0.50, 0.52, 0.54, 1.0];
COL_CAM         = [0.15, 0.15, 0.18, 1.0];  // dark camera domes

// -----------------------------------------------------------------------------
// DERIVED VALUES (do not edit)
// -----------------------------------------------------------------------------

half_l  = hull_length  / 2;
half_w  = hull_width   / 2;
half_h  = hull_height  / 2;

nose_end  =  half_l;
tail_end  = -half_l;

// Belly sits at the flattened bottom of the hull
belly_z = -half_h * (1 - belly_flatten);

// =============================================================================
// MODULES
// =============================================================================

// -----------------------------------------------------------------------------
// Hull - lifting body shape
// Achieved by scaling and intersecting ellipsoids to get a flattened teardrop.
// The belly is further flattened by subtracting a box from below.
// -----------------------------------------------------------------------------
module hull_body() {
    // Primary ellipsoid — the base shape
    // We build it as a scaled sphere: x=length, y=width, z=height
    difference() {
        union() {
            // Main body ellipsoid
            scale([hull_length * 0.48,
                   hull_width  * 0.50,
                   hull_height * 0.50])
                sphere(1);

            // Slightly fuller mid-section blending bulge
            scale([hull_length * 0.28,
                   hull_width  * 0.50,
                   hull_height * 0.46])
                sphere(1);
        }

        // Flatten the belly — remove a slab below belly_z
        translate([0, 0, belly_z - hull_height * belly_flatten - hull_height])
            cube([hull_length * 1.5,
                  hull_width  * 1.5,
                  hull_height * 1.2], center=true);

        // Flatten the top slightly (less aggressive than belly)
        translate([0, 0, half_h * 0.82 + hull_height * 0.3])
            cube([hull_length * 1.5,
                  hull_width  * 1.5,
                  hull_height * 0.6], center=true);

        // Trim the tail — blunt cutoff
        translate([tail_end - hull_length * 0.08, 0, 0])
            cube([hull_length * 0.2,
                  hull_width  * 1.5,
                  hull_height * 1.5], center=true);
    }
}

// -----------------------------------------------------------------------------
// Belly heat shield colouring
// Uses intersection of the hull with a slab to colour the bottom ceramic zone
// without adding any geometry — the belly IS the hull, just a different colour.
// -----------------------------------------------------------------------------
module belly_shield() {
    intersection() {
        hull_body();
        // Slab covering everything below belly_z + band
        translate([0, 0, belly_z - hull_height])
            cube([hull_length * 1.5,
                  hull_width  * 1.5,
                  hull_height * (1 + belly_band)], center=true);
    }
}

// -----------------------------------------------------------------------------
// Engine pod — a single articulating thruster pod
// Oriented so nozzle points -Z (down) when deployed
// pos = [x, y] position on hull underside corners
// cant = toe-out angle around Z axis
// -----------------------------------------------------------------------------
module engine_pod(pos, cant) {
    translate([pos[0], pos[1], belly_z - pod_height * 0.5])
    rotate([0, 0, cant])
    union() {
        // Pod body — rounded box
        color(COL_POD)
        scale([pod_length, pod_width, pod_height])
            hull() {
                translate([ 0.35,  0.25, 0]) sphere(0.25);
                translate([-0.35,  0.25, 0]) sphere(0.25);
                translate([ 0.35, -0.25, 0]) sphere(0.25);
                translate([-0.35, -0.25, 0]) sphere(0.25);
            }

        // Nozzle bell pointing downward
        color(COL_NOZZLE)
        translate([0, 0, -pod_height * 0.55])
            cylinder(h = pod_height * 0.35,
                     r1 = pod_nozzle_r * 1.35,
                     r2 = pod_nozzle_r * 0.55);

        // Nozzle throat
        color(COL_NOZZLE)
        translate([0, 0, -pod_height * 0.55 - pod_height * 0.12])
            cylinder(h = pod_height * 0.14,
                     r  = pod_nozzle_r * 0.52);

        // Pivot arm connecting pod to hull
        color(COL_POD)
        translate([0, 0, pod_height * 0.62])
            cylinder(h = pod_height * 0.5,
                     r  = pod_width  * 0.18);
    }
}

// -----------------------------------------------------------------------------
// All four engine pods placed symmetrically
// -----------------------------------------------------------------------------
module engine_pods() {
    positions = [
        [ pod_offset_x,  pod_offset_y],   // fore starboard
        [ pod_offset_x, -pod_offset_y],   // fore port
        [-pod_offset_x,  pod_offset_y],   // aft  starboard
        [-pod_offset_x, -pod_offset_y]    // aft  port
    ];
    cants = [
        -pod_cant_angle,   // fore starboard — toe out
         pod_cant_angle,   // fore port
        -pod_cant_angle,   // aft  starboard
         pod_cant_angle    // aft  port
    ];

    for (i = [0:3])
        engine_pod(positions[i], cants[i]);
}

// -----------------------------------------------------------------------------
// Landing leg — a single deployed strut with foot pad
// Legs deploy outward and slightly aft
// -----------------------------------------------------------------------------
module landing_leg(side_y, side_x) {
    foot_x = side_x * leg_spread * 0.6;
    foot_y = side_y * leg_spread;
    foot_z = -half_h - leg_height;

    attach_x = side_x * half_l * 0.25;
    attach_y = side_y * half_w * 0.55;
    attach_z = belly_z;

    color(COL_LEG) {
        // Main strut
        hull() {
            translate([attach_x, attach_y, attach_z])
                sphere(leg_width);
            translate([foot_x, foot_y, foot_z])
                sphere(leg_width * 1.1);
        }

        // Diagonal brace
        hull() {
            translate([attach_x * 0.5, attach_y * 0.8, attach_z - 0.3])
                sphere(leg_width * 0.7);
            translate([foot_x, foot_y, foot_z])
                sphere(leg_width * 0.7);
        }

        // Foot pad
        translate([foot_x, foot_y, foot_z - leg_width * 0.5])
            cylinder(h = leg_width * 0.6,
                     r  = leg_width * 2.8);
    }
}

// -----------------------------------------------------------------------------
// All four landing legs
// -----------------------------------------------------------------------------
module landing_legs() {
    landing_leg( 1,  1);   // fore starboard
    landing_leg(-1,  1);   // aft  starboard
    landing_leg( 1, -1);   // fore port
    landing_leg(-1, -1);   // aft  port
}

// -----------------------------------------------------------------------------
// IDSS docking port collar
// axis = "Z+" dorsal, "Z-" ventral, "X+" nose
// -----------------------------------------------------------------------------
module idss_port(radius, collar_h, axis) {
    color(COL_PORT)
    rotate(axis == "Z+" ? [0,   0, 0] :
           axis == "Z-" ? [180, 0, 0] :
           axis == "X+" ? [0,  90, 0] : [0, 0, 0])
    union() {
        // Outer collar ring
        difference() {
            cylinder(h = collar_h, r = radius * 1.28);
            translate([0, 0, -0.01])
                cylinder(h = collar_h + 0.02, r = radius * 0.72);
        }

        // Tunnel stub
        cylinder(h = collar_h * 0.55, r = radius * 0.72);

        // Capture petal ring (3 petals, simplified as fins)
        for (a = [0, 120, 240])
            rotate([0, 0, a])
            translate([radius * 0.90, 0, collar_h * 0.6])
                scale([0.08, 0.18, 0.22])
                    sphere(1);

        // Alignment marks (4 small bumps)
        for (a = [0, 90, 180, 270])
            rotate([0, 0, a])
            translate([radius * 1.18, 0, collar_h * 0.5])
                sphere(radius * 0.06);
    }
}

// -----------------------------------------------------------------------------
// Dorsal (roof) IDSS port — primary structural port
// Used for: space elevator attachment, station docking (approach from below)
// -----------------------------------------------------------------------------
module dorsal_port() {
    // Find approximate hull top at dorsal_port_x
    // Hull top z ≈ half_h * 0.78 at x=0, tapers toward nose/tail
    top_z = half_h * 0.72;
    translate([dorsal_port_x, 0, top_z])
        idss_port(dorsal_port_r, dorsal_port_h, "Z+");
}

// -----------------------------------------------------------------------------
// Nose IDSS port — secondary active port
// Used for: ship-to-ship docking
// -----------------------------------------------------------------------------
module nose_port() {
    // Nose tip x position (hull tapers here)
    nose_x = half_l * 0.88;
    translate([nose_x, 0, 0])
        idss_port(nose_port_r, nose_port_h, "X+");
}

// -----------------------------------------------------------------------------
// RCS thruster cluster block
// Each cluster has 4 nozzles (±X and ±Y) for full 6DOF control
// -----------------------------------------------------------------------------
module rcs_cluster(pos, rot) {
    translate(pos)
    rotate(rot)
    color(COL_RCS) {
        // Block body
        cube([rcs_box_size, rcs_box_size, rcs_box_size * 0.6], center=true);

        // Four nozzles: +X, -X, +Y, -Y
        for (dir = [[1,0,0], [-1,0,0], [0,1,0], [0,-1,0]])
            translate(dir * rcs_box_size * 0.5)
            rotate([0, dir[1] != 0 ? (dir[1] > 0 ? -90 : 90) : 0,
                       dir[0] != 0 ? (dir[0] > 0 ?  90 :-90) : 0])
                cylinder(h = rcs_nozzle_l, r = rcs_nozzle_r);
    }
}

// -----------------------------------------------------------------------------
// All RCS clusters — 4 groups of 2 (fore/aft, port/starboard)
// Gives full 6DOF: pitch, yaw, roll, translate X/Y/Z
// -----------------------------------------------------------------------------
module rcs_clusters() {
    fw =  half_l * 0.55;   // forward cluster longitudinal pos
    aw = -half_l * 0.50;   // aft cluster
    sw =  half_w * 0.62;   // starboard lateral pos
    pw = -half_w * 0.62;   // port lateral pos
    zh =  half_h * 0.30;   // vertical position

    // Forward clusters (port & starboard flanks)
    rcs_cluster([fw,  sw, zh], [0, 0,  0]);
    rcs_cluster([fw,  pw, zh], [0, 0,  0]);

    // Aft clusters
    rcs_cluster([aw,  sw, zh], [0, 0,  0]);
    rcs_cluster([aw,  pw, zh], [0, 0,  0]);

    // Dorsal fore/aft for pitch trim
    rcs_cluster([fw,  0,  half_h * 0.65], [0, 0, 0]);
    rcs_cluster([aw,  0,  half_h * 0.65], [0, 0, 0]);
}

// -----------------------------------------------------------------------------
// Camera cluster — flush-mounted navigation/docking cameras
// -----------------------------------------------------------------------------
module camera_dome(pos, point_dir) {
    translate(pos)
    color(COL_CAM)
        sphere(cam_r);
}

// -----------------------------------------------------------------------------
// Camera clusters around the hull
// Forward-facing, aft-facing, belly (landing), dorsal (docking approach)
// -----------------------------------------------------------------------------
module cameras() {
    // Forward facing — navigation, docking approach
    camera_dome([half_l * 0.80,  half_w * 0.18,  half_h * 0.20], [1,0,0]);
    camera_dome([half_l * 0.80, -half_w * 0.18,  half_h * 0.20], [1,0,0]);
    camera_dome([half_l * 0.75,  0,               half_h * 0.40], [1,0,0]);

    // Aft facing
    camera_dome([tail_end + 0.4,  half_w * 0.20, half_h * 0.20], [-1,0,0]);
    camera_dome([tail_end + 0.4, -half_w * 0.20, half_h * 0.20], [-1,0,0]);

    // Belly — landing cameras
    camera_dome([ half_l * 0.20,  half_w * 0.30, belly_z + 0.05], [0,0,-1]);
    camera_dome([ half_l * 0.20, -half_w * 0.30, belly_z + 0.05], [0,0,-1]);
    camera_dome([-half_l * 0.15,  half_w * 0.28, belly_z + 0.05], [0,0,-1]);
    camera_dome([-half_l * 0.15, -half_w * 0.28, belly_z + 0.05], [0,0,-1]);

    // Dorsal — for docking approach from below / elevator
    camera_dome([dorsal_port_x + 1.2,  0.5, half_h * 0.70], [0,0,1]);
    camera_dome([dorsal_port_x + 1.2, -0.5, half_h * 0.70], [0,0,1]);

    // Side cameras — situational awareness
    camera_dome([ half_l * 0.10,  half_w * 0.50, 0], [0, 1, 0]);
    camera_dome([ half_l * 0.10, -half_w * 0.50, 0], [0,-1, 0]);
    camera_dome([-half_l * 0.15,  half_w * 0.50, 0], [0, 1, 0]);
    camera_dome([-half_l * 0.15, -half_w * 0.50, 0], [0,-1, 0]);
}

// =============================================================================
// ASSEMBLY
// =============================================================================

// Hull — rendered in two passes:
// 1. Full hull in alloy colour
// 2. Belly zone overdrawn in ceramic colour (same geometry, intersection-clipped)
if (SHOW_HULL)
    color(COL_HULL)
        hull_body();

if (SHOW_BELLY)
    color(COL_BELLY)
        belly_shield();

// Engine pods
if (SHOW_ENGINE_PODS)
    engine_pods();

// Landing legs
if (SHOW_LANDING_LEGS)
    landing_legs();

// Dorsal IDSS port (primary — space elevator / station)
if (SHOW_DORSAL_PORT)
    dorsal_port();

// Nose IDSS port (secondary — ship-to-ship)
if (SHOW_NOSE_PORT)
    nose_port();

// RCS clusters
if (SHOW_RCS)
    rcs_clusters();

// Camera clusters
if (SHOW_CAMERAS)
    cameras();

// =============================================================================
// REFERENCE GEOMETRY (comment out for export)
// =============================================================================

// Ground plane reference
// translate([0, 0, -half_h - leg_height - 0.01])
//     color([0.3, 0.3, 0.3, 0.3])
//         cube([hull_length * 1.8, hull_width * 2.2, 0.02], center=true);

// Axis indicators
// color("red")   translate([half_l + 1, 0, 0]) sphere(0.15);  // +X = nose
// color("green") translate([0, half_w + 1, 0]) sphere(0.15);  // +Y = starboard
// color("blue")  translate([0, 0, half_h + 1]) sphere(0.15);  // +Z = dorsal
