// =============================================================================
// Apeiron Utility Drone + Drone Cradle Interface (DCI)
// =============================================================================
// A cold-gas RCS utility drone/ROV for close-proximity operations around
// Apeiron spacecraft. Includes the retractable hull cradle (DCI) that
// provides power, data, and physical retention when stowed.
//
// Propulsion: Cold gas RCS (N2) for translation + reaction wheels for attitude
// Sensors:    Forward camera cluster, proximity LIDAR ring
// Power:      Internal LiPo battery, charged via DCI contact plate
//
// Two models in this file — render separately using the RENDER_* flags.
//
// Requires OpenSCAD 2021.01 or later.
// =============================================================================

// -----------------------------------------------------------------------------
// RENDER FLAGS — set one true at a time, or both for assembly view
// -----------------------------------------------------------------------------
RENDER_DRONE  = true;
RENDER_CRADLE = true;
RENDER_STOWED = false;  // show drone sitting in cradle (overrides above positions)

// -----------------------------------------------------------------------------
// DRONE PARAMETERS
// -----------------------------------------------------------------------------

// Overall body
drone_w      = 0.32;   // width  (X) metres
drone_d      = 0.32;   // depth  (Y) metres
drone_h      = 0.20;   // height (Z) metres
corner_r     = 0.045;  // body corner rounding radius

// Cold gas RCS pods
// Eight thrusters: ±X, ±Y on four corner pods (gives full 6DOF translation)
rcs_pod_r    = 0.025;  // RCS pod cylinder radius
rcs_pod_l    = 0.055;  // RCS pod length
rcs_nozzle_r = 0.010;  // individual nozzle radius
rcs_nozzle_l = 0.018;  // nozzle protrusion

// Reaction wheel housing (internal, shown as inset panel)
rw_w         = 0.10;   // reaction wheel housing width
rw_h         = 0.015;  // inset depth of panel
rw_r         = 0.008;  // corner radius of panel

// Camera cluster (forward face)
cam_main_r   = 0.022;  // main navigation camera radius
cam_main_d   = 0.012;  // main camera protrusion
cam_aux_r    = 0.010;  // auxiliary camera radius
cam_aux_d    = 0.008;
lidar_r      = 0.018;  // LIDAR ring outer radius
lidar_w      = 0.008;  // LIDAR ring width

// Docking face (aft face — mates with cradle)
dock_face_inset = 0.004;   // recessed contact plate inset
contact_r       = 0.006;   // individual electrical contact pin radius
contact_h       = 0.003;   // contact pin protrusion
latch_r         = 0.018;   // mechanical latch receiver radius
latch_depth     = 0.012;   // latch socket depth

// Status LED strip (along equator)
led_w        = 0.005;
led_h        = 0.003;

// -----------------------------------------------------------------------------
// CRADLE PARAMETERS (Drone Cradle Interface — DCI)
// -----------------------------------------------------------------------------

// Cradle outer dimensions — fits into mothership hull
cradle_w     = drone_w + 0.025;   // slightly wider than drone
cradle_d     = drone_d + 0.025;
cradle_h     = drone_h + 0.018;   // slightly taller
cradle_wall  = 0.012;             // wall thickness
cradle_depth = drone_d * 0.65;    // how deep drone sits in cradle

// Hatch (flush cover that closes when drone is deployed)
hatch_thick  = 0.008;             // hatch panel thickness
hatch_gap    = 0.002;             // gap around hatch edge

// Latch pins (two, port and starboard) — spring-loaded, engage drone receiver
latch_pin_r  = latch_r * 0.85;
latch_pin_l  = latch_depth + 0.006;

// Power/data contact plate
contact_plate_w  = 0.08;
contact_plate_h  = 0.04;
contact_plate_d  = 0.004;

// Ejection spring housing (pushes drone clear on release)
spring_r     = 0.015;
spring_h     = 0.020;

// Alignment guides (tapered rails that guide drone into cradle)
guide_l      = cradle_d * 0.55;
guide_w      = 0.010;
guide_h      = 0.014;

// -----------------------------------------------------------------------------
// COLOURS
// -----------------------------------------------------------------------------
COL_BODY     = [0.82, 0.84, 0.86, 1.0];  // light alloy
COL_DARK     = [0.20, 0.22, 0.24, 1.0];  // dark panels / camera faces
COL_NOZZLE   = [0.35, 0.32, 0.28, 1.0];  // exhaust metal
COL_CONTACT  = [0.75, 0.68, 0.20, 1.0];  // gold electrical contacts
COL_LED      = [0.20, 0.80, 0.40, 1.0];  // green status LED
COL_CRADLE   = [0.55, 0.58, 0.62, 1.0];  // mothership hull colour
COL_HATCH    = [0.58, 0.61, 0.65, 1.0];  // hatch panel
COL_LATCH    = [0.65, 0.65, 0.60, 1.0];  // latch mechanism

// -----------------------------------------------------------------------------
// DETAIL LEVEL
// -----------------------------------------------------------------------------
$fn = 40;

// =============================================================================
// DRONE MODULES
// =============================================================================

// -----------------------------------------------------------------------------
// Drone body — rounded rectangular box
// Oriented: +Y = forward (camera face), -Y = aft (docking face), +Z = up
// -----------------------------------------------------------------------------
module drone_body() {
    color(COL_BODY)
    hull() {
        // Eight corner spheres define the rounded box
        for (sx = [-1, 1], sy = [-1, 1], sz = [-1, 1])
            translate([sx * (drone_w/2 - corner_r),
                       sy * (drone_d/2 - corner_r),
                       sz * (drone_h/2 - corner_r)])
                sphere(corner_r);
    }
}

// -----------------------------------------------------------------------------
// Single RCS pod — mounts on corner, fires ±X or ±Y
// -----------------------------------------------------------------------------
module rcs_pod(pos, rot) {
    translate(pos)
    rotate(rot)
    union() {
        // Pod body cylinder
        color(COL_BODY)
        rotate([90, 0, 0])
            cylinder(h = rcs_pod_l, r = rcs_pod_r, center=true);

        // Two opposing nozzles
        color(COL_NOZZLE)
        for (side = [-1, 1])
            translate([0, side * (rcs_pod_l/2 + rcs_nozzle_l/2), 0])
            rotate([90, 0, 0])
                cylinder(h = rcs_nozzle_l,
                         r1 = rcs_nozzle_r * 1.4,
                         r2 = rcs_nozzle_r * 0.7,
                         center=true);
    }
}

// -----------------------------------------------------------------------------
// All RCS pods — four corner pods, each firing in two directions
// Gives full 6DOF: translate X, Y, Z and roll, pitch, yaw
// -----------------------------------------------------------------------------
module rcs_pods() {
    pod_x = drone_w/2 + rcs_pod_r * 0.5;
    pod_z = drone_h/2 * 0.30;

    // Fore starboard — fires ±X
    rcs_pod([pod_x,  drone_d * 0.20, pod_z], [0, 0,  0]);
    // Fore port — fires ±X
    rcs_pod([-pod_x, drone_d * 0.20, pod_z], [0, 0,  0]);
    // Aft starboard — fires ±X
    rcs_pod([pod_x, -drone_d * 0.20, pod_z], [0, 0,  0]);
    // Aft port — fires ±X
    rcs_pod([-pod_x,-drone_d * 0.20, pod_z], [0, 0,  0]);

    // Top fore/aft — fires ±Y (forward/back translation and pitch)
    pod_y_z = drone_h/2 + rcs_pod_r * 0.5;
    rcs_pod([ drone_w * 0.20, 0, pod_y_z], [0, 90, 0]);
    rcs_pod([-drone_w * 0.20, 0, pod_y_z], [0, 90, 0]);
}

// -----------------------------------------------------------------------------
// Forward sensor face — camera cluster and LIDAR ring
// -----------------------------------------------------------------------------
module sensor_face() {
    face_y = drone_d/2;

    // Dark sensor panel background
    color(COL_DARK)
    translate([0, face_y - 0.002, 0])
    scale([drone_w * 0.72, 0.004, drone_h * 0.72])
        hull() {
            for (sx=[-0.4, 0.4], sz=[-0.4, 0.4])
                translate([sx, 0, sz]) sphere(0.5);
        }

    // Main navigation camera (centre)
    color(COL_DARK)
    translate([0, face_y + cam_main_d/2, 0])
        cylinder(h = cam_main_d, r = cam_main_r, center=true);

    // Lens glint
    color([0.6, 0.8, 1.0, 0.9])
    translate([0, face_y + cam_main_d, cam_main_r * 0.15])
        sphere(cam_main_r * 0.35);

    // Auxiliary cameras (stereo pair for depth)
    for (sx = [-1, 1])
        color(COL_DARK)
        translate([sx * drone_w * 0.22, face_y + cam_aux_d/2, drone_h * 0.10])
            cylinder(h = cam_aux_d, r = cam_aux_r, center=true);

    // LIDAR proximity ring (around the sensor face perimeter)
    color([0.30, 0.30, 0.28])
    translate([0, face_y + lidar_w/2, 0])
    difference() {
        scale([1, lidar_w/(lidar_r*2), 1])
            cylinder(h = lidar_r*2, r = lidar_r, center=true);
        cylinder(h = lidar_r*2 + 0.01,
                 r = lidar_r - 0.006, center=true);
    }
}

// -----------------------------------------------------------------------------
// Aft docking face — mechanical latch receivers + electrical contacts
// Mates with cradle DCI
// -----------------------------------------------------------------------------
module docking_face() {
    face_y = -drone_d/2;

    // Recessed contact plate
    color(COL_DARK)
    translate([0, face_y + dock_face_inset/2, 0])
    scale([contact_plate_w * 1.8, dock_face_inset, contact_plate_h * 1.8])
        hull() {
            for (sx=[-0.4, 0.4], sz=[-0.4, 0.4])
                translate([sx, 0, sz]) sphere(0.5);
        }

    // Electrical contact pins (2x3 grid)
    color(COL_CONTACT)
    for (cx = [-1, 0, 1], cz = [-1, 1])
        translate([cx * 0.018,
                   face_y + contact_h/2,
                   cz * 0.012])
            cylinder(h = contact_h, r = contact_r, center=true);

    // Mechanical latch receivers (port and starboard)
    color(COL_DARK)
    for (sx = [-1, 1])
        translate([sx * drone_w * 0.28, face_y - latch_depth/2, 0])
        rotate([90, 0, 0])
            cylinder(h = latch_depth, r = latch_r, center=true);
}

// -----------------------------------------------------------------------------
// Reaction wheel panel insets (port, starboard, top)
// Visible as recessed rectangular panels — wheels are internal
// -----------------------------------------------------------------------------
module reaction_wheel_panels() {
    color(COL_DARK)
    // Port side
    translate([-drone_w/2 + rw_h/2, 0, 0])
    rotate([0, 90, 0])
        hull() {
            for (sy=[-0.4, 0.4], sz=[-0.4, 0.4])
                translate([sy * rw_w * 0.4, sz * rw_w * 0.4, 0])
                    cylinder(h=rw_h, r=rw_r, center=true);
        }

    // Starboard side
    translate([drone_w/2 - rw_h/2, 0, 0])
    rotate([0, 90, 0])
        hull() {
            for (sy=[-0.4, 0.4], sz=[-0.4, 0.4])
                translate([sy * rw_w * 0.4, sz * rw_w * 0.4, 0])
                    cylinder(h=rw_h, r=rw_r, center=true);
        }

    // Top
    translate([0, 0, drone_h/2 - rw_h/2])
    rotate([0, 0, 0])
        hull() {
            for (sx=[-0.4, 0.4], sy=[-0.4, 0.4])
                translate([sx * rw_w * 0.4, sy * rw_w * 0.4, 0])
                    cylinder(h=rw_h, r=rw_r, center=true);
        }
}

// -----------------------------------------------------------------------------
// Status LED strip along the equator
// -----------------------------------------------------------------------------
module status_leds() {
    color(COL_LED)
    for (sx = [-1, 1])
        translate([sx * (drone_w/2 - 0.001), 0, -drone_h/2 + 0.020])
            cube([led_w, drone_d * 0.55, led_h], center=true);
}

// -----------------------------------------------------------------------------
// Complete drone assembly
// -----------------------------------------------------------------------------
module drone(pos=[0,0,0], rot=[0,0,0]) {
    translate(pos)
    rotate(rot)
    union() {
        drone_body();
        rcs_pods();
        sensor_face();
        docking_face();
        reaction_wheel_panels();
        status_leds();
    }
}

// =============================================================================
// CRADLE MODULES
// =============================================================================

// -----------------------------------------------------------------------------
// Cradle body — recessed into mothership hull
// Open on the +Y face (flush with hull surface when hatch is closed)
// -----------------------------------------------------------------------------
module cradle_body() {
    color(COL_CRADLE)
    difference() {
        // Outer shell
        hull() {
            for (sx=[-1,1], sz=[-1,1])
                translate([sx*(cradle_w/2 - cradle_wall),
                           0,
                           sz*(cradle_h/2 - cradle_wall)])
                    cylinder(h=cradle_d, r=cradle_wall, center=true);
        }

        // Interior void — drone-shaped pocket
        translate([0, cradle_wall * 0.5, 0])
        hull() {
            for (sx=[-1,1], sz=[-1,1])
                translate([sx*(drone_w/2 - corner_r + 0.003),
                           0,
                           sz*(drone_h/2 - corner_r + 0.003)])
                    sphere(corner_r + 0.003);
        }

        // Open face (+Y, flush with hull)
        translate([0, cradle_d/2 + 0.01, 0])
            cube([cradle_w, cradle_d * 0.6, cradle_h], center=true);
    }
}

// -----------------------------------------------------------------------------
// Alignment guide rails — tapered, guide drone into correct position
// -----------------------------------------------------------------------------
module alignment_guides() {
    color(COL_CRADLE)
    for (sx = [-1, 1])
        translate([sx * (drone_w/2 + guide_w/2 - 0.002), -cradle_depth/2, 0])
        hull() {
            // Tapered entry (wide at opening)
            translate([0,  guide_l/2, 0])
                cube([guide_w, 0.002, guide_h * 1.6], center=true);
            // Tight at back
            translate([0, -guide_l/2, 0])
                cube([guide_w, 0.002, guide_h], center=true);
        }
}

// -----------------------------------------------------------------------------
// Spring ejector — compressed when drone is seated, releases on command
// -----------------------------------------------------------------------------
module ejection_spring() {
    color(COL_CRADLE)
    translate([0, -cradle_depth + spring_h/2, 0])
        cylinder(h=spring_h, r=spring_r, center=true);

    // Spring coil suggestion (decorative)
    color([0.6, 0.6, 0.55])
    for (i = [0:4])
        translate([0, -cradle_depth + spring_h * 0.15 + i * spring_h * 0.14, 0])
        rotate([90, 0, 0])
        difference() {
            cylinder(h=0.003, r=spring_r * 0.85);
            cylinder(h=0.004, r=spring_r * 0.55);
        }
}

// -----------------------------------------------------------------------------
// Power/data contact plate — on the aft wall of the cradle
// Mates with drone docking face contacts
// -----------------------------------------------------------------------------
module cradle_contact_plate() {
    // Plate body
    color(COL_DARK)
    translate([0, -cradle_depth + contact_plate_d/2, 0])
    hull() {
        for (sx=[-0.4,0.4], sz=[-0.4,0.4])
            translate([sx*contact_plate_w*0.4,
                       0,
                       sz*contact_plate_h*0.4])
                cylinder(h=contact_plate_d, r=0.004, center=true);
    }

    // Contact pins (matching drone pin layout)
    color(COL_CONTACT)
    for (cx = [-1, 0, 1], cz = [-1, 1])
        translate([cx * 0.018,
                   -cradle_depth + contact_plate_d + contact_h/2,
                   cz * 0.012])
            cylinder(h=contact_h, r=contact_r, center=true);
}

// -----------------------------------------------------------------------------
// Latch pins — spring-loaded, engage drone latch receivers
// -----------------------------------------------------------------------------
module latch_pins() {
    color(COL_LATCH)
    for (sx = [-1, 1])
        translate([sx * drone_w * 0.28,
                   -cradle_depth * 0.35,
                   0])
        rotate([90, 0, 0])
            cylinder(h=latch_pin_l, r=latch_pin_r, center=true);
}

// -----------------------------------------------------------------------------
// Hatch — flush panel that covers the cradle opening when drone is deployed
// Shown open (rotated) in assembly; would be flush with hull when closed
// -----------------------------------------------------------------------------
module hatch(open=true) {
    hatch_w = cradle_w - hatch_gap * 2;
    hatch_h_dim = cradle_h - hatch_gap * 2;

    color(COL_HATCH)
    if (open) {
        // Hatch shown swung open (rotated 90° around bottom edge)
        translate([0, -hatch_gap, -cradle_h/2])
        rotate([-90, 0, 0])
        translate([0, 0, hatch_thick/2])
            cube([hatch_w, hatch_h_dim, hatch_thick], center=true);
    } else {
        // Hatch closed — flush with cradle opening
        translate([0, cradle_d * 0.28 + hatch_thick/2, 0])
            cube([hatch_w, hatch_thick, hatch_h_dim], center=true);
    }
}

// -----------------------------------------------------------------------------
// Complete cradle assembly
// Positioned so the open face is at Y=0 (flush with hull surface)
// -----------------------------------------------------------------------------
module cradle(pos=[0,0,0], hatch_open=true) {
    translate(pos)
    union() {
        cradle_body();
        alignment_guides();
        ejection_spring();
        cradle_contact_plate();
        latch_pins();
        hatch(open=hatch_open);
    }
}

// =============================================================================
// ASSEMBLY
// =============================================================================

// Spacing for side-by-side view
drone_display_x  =  RENDER_STOWED ? 0 : (RENDER_CRADLE ?  0.45 : 0);
cradle_display_x =  RENDER_STOWED ? 0 : (RENDER_DRONE  ? -0.45 : 0);

if (RENDER_STOWED) {
    // Drone shown seated in cradle
    cradle([0, 0, 0], hatch_open=true);
    // Drone positioned inside cradle
    drone([0, -cradle_depth * 0.45, 0]);

} else {
    // Side by side display
    if (RENDER_DRONE)
        drone([drone_display_x, 0, 0]);

    if (RENDER_CRADLE)
        cradle([cradle_display_x, 0, 0], hatch_open=true);
}

// =============================================================================
// NOTES FOR APEIRON INTEGRATION
// =============================================================================
//
// Drone physical properties (approximate, to be refined):
//   mass:             8.5 kg (dry, no propellant)
//   cold gas budget:  ~0.5 kg N2 (gives ~8 m/s delta-V for close-proximity ops)
//   reaction wheels:  3x orthogonal, ~0.005 N·m·s each
//   camera:           main nav (1080p), stereo aux pair, LIDAR proximity ring
//   battery:          ~200 Wh, recharged via DCI contact plate
//   operating range:  ~50m from mothership before tether/comms degrades
//
// DCI (Drone Cradle Interface) integration:
//   - Recessed into mothership hull — zero protrusion when hatch closed
//   - Hatch is flush with hull surface, opens outward/downward before launch
//   - Ejection spring provides ~0.3 m/s separation velocity on release
//   - Latch pins retract via solenoid on release command
//   - Contact plate provides: 28V power, SpaceWire data bus
//   - Multiple DCIs can be placed on different hull faces (belly, dorsal, flanks)
//   - DCI node name in glTF: "dci_01", "dci_02" etc.
//   - Drone docks autonomously using LIDAR ring + aft camera on mothership
//
// Propulsion summary:
//   Cold gas (N2): 8 thrusters on 4 corner pods, ±X translation and yaw
//   Cold gas (N2): 4 additional top-mounted nozzles, ±Y translation and pitch
//   Reaction wheels: 3 orthogonal, fine attitude hold and roll control
//   No combustion products — safe for use near sensitive surfaces and airlocks
