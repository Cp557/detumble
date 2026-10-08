# Third-Party Software and Assets

The orbital spacecraft uses an adapted NASA ICECube model, with a project-authored fallback. The sidebar hardware schematic is project-authored; a previously used CAD adaptation is retained as a reference asset, attributed below. The geomagnetic data and adapted synthesis algorithm are also attributed below.

CMake downloads pinned copies of these dependencies during configuration:

| Dependency | Purpose | License |
| --- | --- | --- |
| Eigen 5.0.1 | Vectors, matrices, quaternions, and decompositions | MPL 2.0 and compatible licenses |
| Catch2 3.15.3 | Unit and integration testing | Boost Software License 1.0 |
| raylib 6.0 | Windowing, input, 3D rendering, and GLB loading | zlib/libpng |
| Dear ImGui 1.92.7 | Desktop user interface | MIT |
| rlImGui `3bc5731` | raylib and Dear ImGui integration | zlib/libpng |

Their complete license texts are included in the source archives fetched by CMake. The generated `assets/models/detumble_3u.glb` is produced from Blender primitives by `assets/scripts/create_detumble_3u.py` and contains no external geometry or textures.

## Spacecraft and sidebar CAD

`assets/models/suchai_cutaway.glb` contains selected, transformed, simplified, and recolored geometry from SPEL's [SUCHAI-II CAD repository](https://github.com/spel-uchile/CAD_SUCHAI_II), revision `cd587d140626b171df6efd11ef67736fa53c759e`. The repository's MIT license, copyright (c) 2017 SPEL, is retained in `assets/models/SUCHAI-II-LICENSE.txt` for the retained reference asset. It is no longer loaded or packaged by the viewer. Source checksum, adaptation details, and regeneration commands are in [the model README](assets/models/README.md). The live magnetic hardware remains project-authored and the view is not a reproduction of SUCHAI-II's flight configuration.

`assets/models/icecube_3u.glb` is a decoded, centered, uniformly scaled adaptation of [NASA's ICECube model](https://science.nasa.gov/3d-resources/cubesat-icecube/), credited to NASA/Christopher R. Meaney. It is used under [NASA's media usage guidelines](https://www.nasa.gov/nasa-brand-center/images-and-media/). It is the default orbital spacecraft. Attribution and adaptation details are retained in `assets/models/ICECube-NOTICE.txt` and copied alongside the runtime model. The comparison render `docs/media/icecube-preview.png` uses the same source. No NASA endorsement is implied.

## Geomagnetic model

IGRF-14 is developed by the International Association of Geomagnetism and Aeronomy
(IAGA), from data contributed by participating institutions. The official
coefficient table is bundled unmodified in `assets/geomagnetic/igrf14coeffs.txt`.
Source, checksum, and regeneration instructions are recorded in that directory.

The Schmidt quasi-normal synthesis recurrence in `src/magnetic_field.cpp` is
adapted from the official NOAA/British Geological Survey `igrf14syn` routine,
updated by Ciaran Beggan in November 2024, following earlier work by Susan
Macmillan and William Brown. This adaptation uses only the geocentric main-field
synthesis for 2025–2030. Independent reference fixtures were generated with the
original Fortran program.

[NOAA's IGRF page](https://www.ncei.noaa.gov/products/international-geomagnetic-reference-field)
states that its supplied software is public domain. The coefficients retain their
IAGA attribution; no claim is made that Detumble authored the geomagnetic data or
the adapted recurrence.
