"""Build the adapted SUCHAI-II sidebar asset from tessellated CAD geometry.

Usage: blender --background --python assets/scripts/create_suchai_cutaway.py -- INPUT.json
See assets/models/README.md for pinned source, attribution, and conversion steps.
"""

from pathlib import Path
import sys
import json
import math

import bpy
from mathutils import Vector, Matrix

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
if "--" not in sys.argv or len(sys.argv) <= sys.argv.index("--") + 1:
    raise RuntimeError("Pass the tessellated JSON path after --")
source = Path(sys.argv[sys.argv.index("--") + 1])
with source.open() as stream:
    parts = json.load(stream)
frames = [p for p in parts if "SIDE_FRAME_007BOTTOM" in p["name"]]
centers = [
    Vector([(p["bounds"][i] + p["bounds"][i + 3]) / 2 for i in range(3)])
    for p in frames
]
if len(centers) != 2:
    raise RuntimeError("Expected two bottom side-frame parts in the pinned assembly")
y = centers[1] - centers[0]
y.z = 0
y.normalize()
angle = math.atan2(-y.x, y.y)
rotation = Matrix.Rotation(-angle, 3, "Z")
frame_parts = [p for p in parts if "SIDE_FRAME_" in p["name"].split("/")[-1]]
lo = Vector([min(p["bounds"][i] for p in frame_parts) for i in range(3)])
hi = Vector([max(p["bounds"][i + 3] for p in frame_parts) for i in range(3)])
origin = Vector(
    (
        (centers[0].x + centers[1].x) / 2,
        (centers[0].y + centers[1].y) / 2,
        (lo.z + hi.z) / 2,
    )
)
scale = 0.34 / (hi.z - lo.z)
print("TRANSFORM", angle, tuple(origin), scale, flush=True)
materials = {}
for name, color in [
    ("Aluminum", (0.72, 0.77, 0.82)),
    ("PCB", (0.07, 0.32, 0.25)),
    ("Component", (0.12, 0.15, 0.20)),
    ("Connector", (0.58, 0.47, 0.26)),
]:
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*color, 1)
    m.use_nodes = True
    m.node_tree.nodes.get("Principled BSDF").inputs["Base Color"].default_value = (
        *color,
        1,
    )
    materials[name] = m
count = 0
for p in parts:
    name = p["name"]
    short = name.split("/")[-1]
    if not p["vertices"] or not p["triangles"]:
        continue
    verts = [rotation @ (Vector(v) - origin) * scale for v in p["vertices"]]
    if "/ISIS 3U Cubesat Structure/" in name:
        if (
            short.startswith(("SCREW", "NUT", "SPRING", "KILL_SWITCH"))
            or "KS_MECH" in name
        ):
            continue
        # Open the camera-facing side of the real frame, leaving top/bottom stubs.
        if "SIDE_FRAME" in short and sum(v.y for v in verts) / len(verts) < 0:
            faces = [
                f for f in p["triangles"] if all(abs(verts[i].z) > 0.13 for i in f)
            ]
        else:
            faces = p["triangles"]
        mat = materials["Aluminum"]
    else:
        # Retain only lower electronics; the original full stack would hide our rods.
        if sum(v.z for v in verts) / len(verts) > -0.13:
            continue
        sizes = sorted(
            max(v[i] for v in verts) - min(v[i] for v in verts) for i in range(3)
        )
        if sizes[2] < 0.008 or sizes[1] < 0.005:
            continue
        if short.lower().startswith(("screw", "20140506")):
            continue
        faces = p["triangles"]
        if short in ("Board", "bottomlayer", "toplayer"):
            mat = materials["PCB"]
        elif any(x in name for x in ("Samtec", "M80-", "picoblade")):
            mat = materials["Connector"]
        else:
            mat = materials["Component"]
    if not faces:
        continue
    mesh = bpy.data.meshes.new(short)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(f"CAD_{count}_{short}", mesh)
    bpy.context.collection.objects.link(obj)
    mesh.materials.append(mat)
    if len(mesh.polygons) > 800:
        bpy.context.view_layer.objects.active = obj
        modifier = obj.modifiers.new("Sidebar detail", "DECIMATE")
        modifier.ratio = max(0.12, 400 / len(mesh.polygons))
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    mesh = obj.data
    # Bake directional shading into vertex colors for the viewer's simple shader.
    light = Vector((-0.4, -0.6, 0.8)).normalized()
    colors = mesh.color_attributes.new(
        name="Shading", type="BYTE_COLOR", domain="CORNER"
    )
    for polygon in mesh.polygons:
        brightness = 0.58 + 0.42 * max(0, polygon.normal.dot(light))
        for index in polygon.loop_indices:
            colors.data[index].color = (brightness, brightness, brightness, 1)
    mesh.color_attributes.active_color = colors
    count += 1
print(
    "OBJECTS",
    count,
    "TRIANGLES",
    sum(len(o.data.polygons) for o in bpy.context.scene.objects if o.type == "MESH"),
    flush=True,
)
bpy.ops.export_scene.gltf(
    filepath=str(
        Path(__file__).resolve().parent.parent / "models" / "suchai_cutaway.glb"
    ),
    export_format="GLB",
    export_yup=False,
    export_materials="EXPORT",
    export_vertex_color="ACTIVE",
    export_all_vertex_colors=True,
)
