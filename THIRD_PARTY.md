# Third-Party Assets

## NASA Generic CubeSat 1 RU

The Phase 3 viewer used NASA's **CubeSat - 1 RU Generic** model as a temporary visualization asset:

- Source: [NASA Science 3D Resources](https://science.nasa.gov/3d-resources/cubesat-1-ru-generic/)
- Creator credited by NASA: Christopher R. Meaney
- Original file: `assets/models/cubesat_1u_nasa.glb`
- Uncompressed development copy: `assets/models/cubesat_1u_nasa_uncompressed.glb`

The source GLB uses Draco mesh compression, which raylib 6.0 does not decode. The viewer copy was produced with glTF Transform CLI 4.2.1's lossless `copy` command to remove that compression. Its geometry and materials were otherwise left unchanged.

These assets remain in the repository as attributed development references. The Phase 7 viewer no longer copies, loads, or renders them. Its custom `detumble_3u.glb` is generated entirely from project-authored Blender primitives and materials and does not use NASA geometry or textures. Neither asset defines the simulation's mass properties, dimensions, component positions, or body-frame convention.

Use and redistribution of NASA media remain subject to the [NASA Media Usage Guidelines](https://www.nasa.gov/nasa-brand-center/images-and-media/).
