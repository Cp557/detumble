# Third-Party Software and Assets

Detumble's source code and generated 3U spacecraft model are project-authored. No third-party model or texture is included in the runtime application.

CMake downloads pinned copies of these dependencies during configuration:

| Dependency | Purpose | License |
| --- | --- | --- |
| Eigen 5.0.1 | Vectors, matrices, quaternions, and decompositions | MPL 2.0 and compatible licenses |
| Catch2 3.15.3 | Unit and integration testing | Boost Software License 1.0 |
| raylib 6.0 | Windowing, input, 3D rendering, and GLB loading | zlib/libpng |
| Dear ImGui 1.92.7 | Desktop user interface | MIT |
| rlImGui `3bc5731` | raylib and Dear ImGui integration | zlib/libpng |

Their complete license texts are included in the source archives fetched by CMake. The generated `assets/models/detumble_3u.glb` is produced from Blender primitives by `assets/scripts/create_detumble_3u.py` and contains no external geometry or textures.
