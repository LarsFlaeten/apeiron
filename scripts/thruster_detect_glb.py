import bpy
import mathutils
import re

# =============================================================================
# Apeiron Thruster Extraction Script
# Extracts thruster positions and directions from ref empty objects in Blender
# and outputs a TOML manifest block ready to paste into the spacecraft config.
#
# Naming conventions expected:
#   main_ref / ref_main  -> main engine
#   aux_ref_N            -> auxiliary thruster N (1-8)
#   rcs_ref_N            -> RCS thruster N (1-32)
#
# The +Z axis of each empty is the THRUST direction.
#
# IMPORTANT: set ROOT_OBJECT_NAME to the name of your spacecraft root object.
# All positions are output in model space (relative to that root), so it does
# not matter where the spacecraft sits in the world or whether transforms are
# applied — the relative geometry is always correct.
#
# Run in Blender: Scripting tab -> New -> paste -> Run Script
# Output appears in the System Console (Window -> Toggle System Console on
# Windows, or run Blender from a terminal on Linux to see stdout).
# =============================================================================

# --- Set this to your spacecraft root object name ---
ROOT_OBJECT_NAME = "CM_SM"   # change if your root is named differently

def get_root_inv():
    """Return the inverse world matrix of the spacecraft root, or identity."""
    root = bpy.data.objects.get(ROOT_OBJECT_NAME)
    if root is None:
        print(f"# WARNING: root object '{ROOT_OBJECT_NAME}' not found — "
              "positions will be in world space.")
        return mathutils.Matrix.Identity(4)
    return root.matrix_world.inverted()

def get_model_position(obj, root_inv):
    """Position of obj in model space (relative to spacecraft root)."""
    world_pos = obj.matrix_world.translation.copy()
    return (root_inv @ world_pos.to_4d()).to_3d()

def get_thrust_direction(obj, root_inv):
    """Thrust direction (+Z axis of empty) expressed in model space."""
    mat = obj.matrix_world
    z_world = mathutils.Vector((mat[0][2], mat[1][2], mat[2][2]))
    # Rotate into model space (translation doesn't affect direction).
    z_model = root_inv.to_3x3() @ z_world
    return z_model.normalized()

def fmt_vec3(v, precision=6):
    """Format a Vector as a TOML inline array."""
    return f"[{v.x:.{precision}f}, {v.y:.{precision}f}, {v.z:.{precision}f}]"

def classify_ref(name):
    """
    Classify a ref empty by name.
    Returns (type, id_string, quad) or None if not a ref empty.
    """
    name_lower = name.lower()

    # Main engine (also accept "ref_main" for consistency)
    if name_lower in ("ref_main", "main_ref"):
        return ("main_engine", "main", None)

    # Auxiliary: aux_ref_N
    m = re.match(r"aux_ref_(\d+)$", name_lower)
    if m:
        n = int(m.group(1))
        return ("auxiliary", f"aux_{n}", None)

    # RCS: rcs_ref_N
    m = re.match(r"rcs_ref_(\d+)$", name_lower)
    if m:
        n = int(m.group(1))
        quad = ((n - 1) // 4) + 1
        return ("rcs", f"rcs_{n}", quad)

    return None

def find_exhaust_node(ref_obj):
    """
    Find the exhaust plume child of this ref empty.
    Expects exactly one child that is a mesh (the plume).
    """
    for child in ref_obj.children:
        if child.type == 'MESH':
            return child.name
    return None

def thruster_sort_key(item):
    """Sort thrusters: main first, then aux by number, then rcs by number."""
    thruster_type, name, _, _, _, _, _ = item
    if thruster_type == "main_engine":
        return (0, 0)
    elif thruster_type == "auxiliary":
        m = re.search(r"(\d+)$", name)
        return (1, int(m.group(1)) if m else 0)
    elif thruster_type == "rcs":
        m = re.search(r"(\d+)$", name)
        return (2, int(m.group(1)) if m else 0)
    return (3, 0)

# -----------------------------------------------------------------------------
# Main extraction
# -----------------------------------------------------------------------------

thrusters = []
unrecognised = []

root_inv = get_root_inv()

for obj in bpy.data.objects:
    result = classify_ref(obj.name)
    if result is None:
        # Collect unrecognised objects that look like they might be refs
        if "ref" in obj.name.lower() and obj.type == 'EMPTY':
            unrecognised.append(obj.name)
        continue

    thruster_type, thruster_id, quad = result
    pos   = get_model_position(obj, root_inv)
    direc = get_thrust_direction(obj, root_inv)
    exhaust = find_exhaust_node(obj)

    thrusters.append((thruster_type, thruster_id, quad, pos, direc, exhaust, obj.name))

# Sort for clean output
thrusters.sort(key=thruster_sort_key)

# -----------------------------------------------------------------------------
# Generate TOML output
# -----------------------------------------------------------------------------

lines = []
lines.append("# =============================================================")
lines.append("# Apeiron — Orion Thruster Manifest")
lines.append(f"# Extracted from Blender scene: {bpy.data.filepath or 'unsaved'}")
lines.append(f"# Total thrusters found: {len(thrusters)}")
lines.append("# =============================================================")
lines.append("")

# Type-specific properties
type_props = {
    "main_engine": {
        "thrust_n":       25700.0,
        "isp_s":          316.0,
        "propellant":     "MMH/MON-3",
        "gimballed":      "true",
        "gimbal_axes":    '["pitch", "yaw"]',
        "gimbal_max_deg": 6.0,
        "exhaust_scale":  1.0,
        "plume_color":    "[0.6, 0.8, 1.0]",   # blue-white
        "plume_intensity": 8.0,
    },
    "auxiliary": {
        "thrust_n":       490.0,
        "isp_s":          312.0,
        "propellant":     "MMH/MON-3",
        "gimballed":      "false",
        "exhaust_scale":  0.5,
        "plume_color":    "[1.0, 0.6, 0.2]",   # orange
        "plume_intensity": 3.0,
    },
    "rcs": {
        "thrust_n":       220.0,
        "isp_s":          280.0,
        "propellant":     "MMH/MON-3",
        "gimballed":      "false",
        "exhaust_scale":  0.25,
        "plume_color":    "[0.7, 0.85, 1.0]",  # cold blue-white
        "plume_intensity": 1.5,
    },
}

counts = {"main_engine": 0, "auxiliary": 0, "rcs": 0}

for (thruster_type, thruster_id, quad, pos, direc, exhaust, ref_name) in thrusters:
    counts[thruster_type] += 1
    props = type_props[thruster_type]

    lines.append("[[spacecraft.orion.thrusters]]")
    lines.append(f'id           = "{thruster_id}"')
    lines.append(f'type         = "{thruster_type}"')

    if quad is not None:
        lines.append(f'quad         = {quad}')

    lines.append(f'thrust_n     = {props["thrust_n"]}')
    lines.append(f'isp_s        = {props["isp_s"]}')
    lines.append(f'propellant   = "{props["propellant"]}"')
    lines.append(f'gimballed    = {props["gimballed"]}')

    if thruster_type == "main_engine":
        lines.append(f'gimbal_axes    = {props["gimbal_axes"]}')
        lines.append(f'gimbal_max_deg = {props["gimbal_max_deg"]}')

    lines.append(f'position     = {fmt_vec3(pos)}')
    lines.append(f'direction    = {fmt_vec3(direc)}')
    lines.append(f'ref_node     = "{ref_name}"')

    if exhaust:
        lines.append(f'exhaust_node = "{exhaust}"')
    else:
        lines.append(f'exhaust_node = ""  # WARNING: no mesh child found on ref empty')

    lines.append(f'exhaust_scale    = {props["exhaust_scale"]}')
    lines.append(f'plume_color      = {props["plume_color"]}')
    lines.append(f'plume_intensity  = {props["plume_intensity"]}')
    lines.append("")

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------

lines.append("# =============================================================")
lines.append(f"# Summary: {counts['main_engine']} main, "
             f"{counts['auxiliary']} auxiliary, "
             f"{counts['rcs']} RCS")
lines.append("# =============================================================")

output = "\n".join(lines)
print(output)

# Also write to a file next to the .blend file for convenience
blend_path = bpy.data.filepath
if blend_path:
    import os
    out_path = os.path.splitext(blend_path)[0] + "_thrusters.toml"
    with open(out_path, "w") as f:
        f.write(output)
    print(f"\n# Written to: {out_path}")
else:
    print("\n# WARNING: Blend file not saved — output not written to disk.")

# Report any unrecognised ref empties
if unrecognised:
    print("\n# WARNING: These empties contain 'ref' but were not recognised:")
    for name in unrecognised:
        print(f"#   {name}")