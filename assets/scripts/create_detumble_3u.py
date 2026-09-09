"""Generate Detumble's custom axis-aligned 3U CubeSat GLB.

Run from the repository root:
    blender --background --python assets/scripts/create_detumble_3u.py

The authoritative component placement is generic_3u_visual_config() in C++.
Keep the mirrored dimensions below synchronized with that configuration.
Blender's +X, +Y, and +Z axes are exported without Y-up conversion so they
match Detumble's body +X, +Y, and long-axis +Z convention directly.
"""

from pathlib import Path

import bpy
from mathutils import Vector


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
OUTPUT_PATH = SCRIPT_DIRECTORY.parent / "models" / "detumble_3u.glb"


def material(name, color, metallic=0.0, roughness=0.45):
    result = bpy.data.materials.new(name)
    result.diffuse_color = (*color, 1.0)
    result.use_nodes = True
    principled = result.node_tree.nodes.get("Principled BSDF")
    if principled is None:
        raise RuntimeError(f"Material {name} has no Principled BSDF node")
    principled.inputs["Base Color"].default_value = (*color, 1.0)
    principled.inputs["Metallic"].default_value = metallic
    principled.inputs["Roughness"].default_value = roughness
    return result


ALUMINUM = material("Aluminum", (0.58, 0.63, 0.69), 0.8, 0.25)
PANEL = material("Panel", (0.055, 0.07, 0.09), 0.4, 0.45)
SOLAR = material("SolarCell", (0.025, 0.09, 0.30), 0.25, 0.3)
GOLD = material("GoldTrace", (0.70, 0.44, 0.08), 0.75, 0.24)
BUS = material("InternalBus", (0.18, 0.23, 0.28), 0.35, 0.55)
TORQUER_X = material("TorquerX", (0.90, 0.18, 0.18), 0.15, 0.45)
TORQUER_Y = material("TorquerY", (0.18, 0.78, 0.25), 0.15, 0.45)
TORQUER_Z = material("TorquerZ", (0.16, 0.38, 0.95), 0.15, 0.45)
SENSOR = material("Sensor", (0.20, 0.85, 0.95), 0.2, 0.3)
MAGNETOMETER = material("Magnetometer", (0.95, 0.35, 0.78), 0.2, 0.3)
WHEEL = material("ReactionWheel", (0.82, 0.47, 0.10), 0.75, 0.25)
FASTENER = material("Fastener", (0.025, 0.03, 0.04), 0.85, 0.2)


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)


def add_box(name, dimensions, center, box_material, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(location=center)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel > 0.0:
        modifier = obj.modifiers.new("EdgeSoftening", "BEVEL")
        modifier.width = bevel
        modifier.segments = 2
    obj.data.materials.append(box_material)
    return obj


def align_local_z(obj, axis):
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = Vector(axis).normalized().to_track_quat("Z", "Y")


def add_cylinder(name, center, axis, radius, length, cylinder_material, vertices=24):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=length,
        location=center,
    )
    obj = bpy.context.object
    obj.name = name
    align_local_z(obj, axis)
    obj.data.materials.append(cylinder_material)
    return obj


def add_surface_sensor(name, center, normal):
    obj = add_box(name, (0.016, 0.016, 0.0015), center, SENSOR, 0.0007)
    align_local_z(obj, normal)
    return obj


def add_structure(root):
    for x in (-0.046, 0.046):
        for y in (-0.046, 0.046):
            rail = add_box(
                f"FrameRail_{'P' if x > 0 else 'N'}X_{'P' if y > 0 else 'N'}Y",
                (0.008, 0.008, 0.34),
                (x, y, 0.0),
                ALUMINUM,
                0.0015,
            )
            rail.parent = root

    for z, suffix in ((-0.168, "NegZ"), (0.168, "PosZ")):
        plate = add_box(
            f"EndPlate_{suffix}",
            (0.10, 0.10, 0.004),
            (0.0, 0.0, z),
            ALUMINUM,
            0.001,
        )
        plate.parent = root

        for x in (-0.034, 0.034):
            for y in (-0.034, 0.034):
                fastener = add_cylinder(
                    f"Fastener_{suffix}_{x:+.3f}_{y:+.3f}",
                    (x, y, z + (0.0025 if z > 0 else -0.0025)),
                    (0.0, 0.0, 1.0),
                    0.0025,
                    0.0015,
                    FASTENER,
                    16,
                )
                fastener.parent = root

    side_panels = (
        ("PosX", (0.049, 0.0, 0.0), (0.002, 0.084, 0.316)),
        ("NegX", (-0.049, 0.0, 0.0), (0.002, 0.084, 0.316)),
        ("PosY", (0.0, 0.049, 0.0), (0.084, 0.002, 0.316)),
        ("NegY", (0.0, -0.049, 0.0), (0.084, 0.002, 0.316)),
    )
    for suffix, center, dimensions in side_panels:
        panel = add_box(f"SidePanel_{suffix}", dimensions, center, PANEL, 0.0008)
        panel.parent = root

    bus = add_box(
        "InternalAvionicsBus",
        (0.074, 0.074, 0.25),
        (0.0, 0.0, 0.0),
        BUS,
        0.004,
    )
    bus.parent = root


def add_solar_arrays(root):
    arrays = (
        ("PosX", (0.0505, 0.0, 0.0), (0.001, 0.078, 0.292)),
        ("NegX", (-0.0505, 0.0, 0.0), (0.001, 0.078, 0.292)),
        ("PosY", (0.0, 0.0505, 0.0), (0.078, 0.001, 0.292)),
        ("NegY", (0.0, -0.0505, 0.0), (0.078, 0.001, 0.292)),
    )
    for suffix, center, dimensions in arrays:
        backing = add_box(
            f"SolarArray_{suffix}",
            dimensions,
            center,
            PANEL,
            0.0005,
        )
        backing.parent = root

        outward_axis = 0 if suffix.endswith("X") else 1
        outward_sign = 1.0 if suffix.startswith("Pos") else -1.0
        across_axis = 1 if outward_axis == 0 else 0
        for column in (-1, 1):
            for row in range(6):
                cell_center = [0.0, 0.0, -0.1215 + row * 0.0486]
                cell_center[outward_axis] = outward_sign * 0.0514
                cell_center[across_axis] = column * 0.019
                cell_dimensions = [0.035, 0.035, 0.043]
                cell_dimensions[outward_axis] = 0.0006
                cell = add_box(
                    f"SolarCell_{suffix}_{column:+d}_{row}",
                    tuple(cell_dimensions),
                    tuple(cell_center),
                    SOLAR,
                    0.0004,
                )
                cell.parent = root

    for z in (-0.097, 0.0, 0.097):
        for x in (-0.0518, 0.0518):
            trace = add_box(
                f"SolarTrace_X_{x:+.4f}_{z:+.3f}",
                (0.0004, 0.078, 0.0015),
                (x, 0.0, z),
                GOLD,
            )
            trace.parent = root
        for y in (-0.0518, 0.0518):
            trace = add_box(
                f"SolarTrace_Y_{y:+.4f}_{z:+.3f}",
                (0.078, 0.0004, 0.0015),
                (0.0, y, z),
                GOLD,
            )
            trace.parent = root


def add_adcs_components(root):
    torquers = (
        ("Magnetorquer_X", (0.0, 0.0, 0.065), (1.0, 0.0, 0.0), 0.075, TORQUER_X),
        ("Magnetorquer_Y", (0.0, 0.0, -0.015), (0.0, 1.0, 0.0), 0.075, TORQUER_Y),
        ("Magnetorquer_Z", (0.025, -0.025, 0.0), (0.0, 0.0, 1.0), 0.24, TORQUER_Z),
    )
    for name, center, axis, length, rod_material in torquers:
        rod = add_cylinder(name, center, axis, 0.004, length, rod_material)
        rod.parent = root

    pyramid = 0.7071067811865476
    wheels = (
        ("ReactionWheel_PosX", (0.022, 0.0, -0.065), (pyramid, 0.0, pyramid)),
        ("ReactionWheel_PosY", (0.0, 0.022, -0.025), (0.0, pyramid, pyramid)),
        ("ReactionWheel_NegX", (-0.022, 0.0, 0.015), (-pyramid, 0.0, pyramid)),
        ("ReactionWheel_NegY", (0.0, -0.022, 0.055), (0.0, -pyramid, pyramid)),
    )
    for name, center, axis in wheels:
        wheel = add_cylinder(name, center, axis, 0.018, 0.009, WHEEL, 32)
        wheel.parent = root


def add_sensors(root):
    sensors = (
        ("CSS_PosX", (0.0515, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ("CSS_NegX", (-0.0515, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        ("CSS_PosY", (0.0, 0.0515, 0.0), (0.0, 1.0, 0.0)),
        ("CSS_NegY", (0.0, -0.0515, 0.0), (0.0, -1.0, 0.0)),
        ("CSS_PosZ", (0.0, 0.0, 0.1715), (0.0, 0.0, 1.0)),
        ("CSS_NegZ", (0.0, 0.0, -0.1715), (0.0, 0.0, -1.0)),
    )
    for name, center, normal in sensors:
        sensor = add_surface_sensor(name, center, normal)
        sensor.parent = root

    boom = add_cylinder(
        "MagnetometerBoom",
        (0.054, 0.0, 0.125),
        (1.0, 0.0, 0.0),
        0.0015,
        0.012,
        ALUMINUM,
        12,
    )
    boom.parent = root
    magnetometer = add_box(
        "Magnetometer",
        (0.010, 0.012, 0.012),
        (0.058, 0.0, 0.125),
        MAGNETOMETER,
        0.0015,
    )
    magnetometer.parent = root


def validate_scene(root):
    required_components = {
        "Magnetorquer_X",
        "Magnetorquer_Y",
        "Magnetorquer_Z",
        "CSS_PosX",
        "CSS_NegX",
        "CSS_PosY",
        "CSS_NegY",
        "CSS_PosZ",
        "CSS_NegZ",
        "Magnetometer",
        "ReactionWheel_PosX",
        "ReactionWheel_PosY",
        "ReactionWheel_NegX",
        "ReactionWheel_NegY",
    }
    missing_components = required_components.difference(bpy.data.objects.keys())
    if missing_components:
        raise RuntimeError(
            f"Missing named spacecraft components: {sorted(missing_components)}"
        )
    if root.location.length > 1.0e-9:
        raise RuntimeError("The body-frame root must remain at the model origin")

    bpy.context.view_layer.update()
    corners = [
        obj.matrix_world @ Vector(corner)
        for obj in bpy.context.scene.objects
        if obj.type == "MESH"
        for corner in obj.bound_box
    ]
    minimum = Vector(tuple(min(corner[index] for corner in corners) for index in range(3)))
    maximum = Vector(tuple(max(corner[index] for corner in corners) for index in range(3)))
    dimensions = maximum - minimum
    if dimensions.z <= max(dimensions.x, dimensions.y):
        raise RuntimeError("The generated spacecraft must use body +Z as its long axis")
    print(
        "Validated body-frame origin and model bounds: "
        f"{dimensions.x:.4f} x {dimensions.y:.4f} x {dimensions.z:.4f} m"
    )


def main():
    clear_scene()
    root = bpy.data.objects.new("Detumble_3U_BodyFrame", None)
    bpy.context.collection.objects.link(root)

    add_structure(root)
    add_solar_arrays(root)
    add_adcs_components(root)
    add_sensors(root)

    root["body_frame_origin"] = "center_of_mass"
    root["body_axis_x"] = "+X designated side panel"
    root["body_axis_y"] = "+Y completes right-handed frame"
    root["body_axis_z"] = "+Z long axis toward forward end"

    validate_scene(root)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT_PATH),
        export_format="GLB",
        export_yup=False,
        export_apply=True,
        export_extras=True,
    )
    print(f"Exported {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
