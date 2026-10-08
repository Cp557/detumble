"""Decode NASA's ICECube GLB and normalize it for the orbital viewer.

Run with Blender: blender --background --python assets/scripts/prepare_icecube.py
    -- /tmp/detumble-icecube.glb
"""

import hashlib
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Vector

SOURCE_SHA256 = "7ad1a4be4beb2c1b62b6fc48a9538db6c914d871099681745e188deb1925f31a"


def main():
    source = Path(sys.argv[sys.argv.index("--") + 1])
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("ICECube source checksum differs from the documented asset")

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.gltf(filepath=str(source))
    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    points = [
        obj.matrix_world @ vertex.co for obj in meshes for vertex in obj.data.vertices
    ]
    lower = Vector(tuple(min(point[axis] for point in points) for axis in range(3)))
    upper = Vector(tuple(max(point[axis] for point in points) for axis in range(3)))
    # The imported mesh already has its long axis along Z. Preserve proportions
    # while centering the complete exterior and giving it a 0.34 m length.
    normalization = Matrix.Scale(0.34 / (upper.z - lower.z), 4) @ Matrix.Translation(
        -(lower + upper) * 0.5
    )
    for obj in meshes:
        world = obj.matrix_world.copy()
        obj.data = obj.data.copy()
        obj.data.transform(normalization @ world)
        obj.parent = None
        obj.matrix_world = Matrix.Identity(4)
    for obj in list(bpy.context.scene.objects):
        if obj.type != "MESH":
            bpy.data.objects.remove(obj, do_unlink=True)

    output = Path(__file__).resolve().parent.parent / "models" / "icecube_3u.glb"
    bpy.ops.export_scene.gltf(
        filepath=str(output),
        export_format="GLB",
        export_yup=False,
        export_draco_mesh_compression_enable=False,
        export_materials="EXPORT",
    )
    print(f"Saved centered ICECube exterior in meters: {output}")


if __name__ == "__main__":
    main()
