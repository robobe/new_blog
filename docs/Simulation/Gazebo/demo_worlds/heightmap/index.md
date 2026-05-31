---
title: Gazebo Heightmap Terrain
tags:
    - gazebo
    - harmonic
    - terrain
    - heightmap
---

# Gazebo heightmap terrain

A heightmap creates terrain from a grayscale image. Dark pixels are low points, bright pixels are high points, and Gazebo scales the result using the `<size>` element.

In Gazebo Harmonic, use the SDFormat `<heightmap>` geometry inside a normal static model. Put the same heightmap geometry in both `<collision>` and `<visual>` so physics and rendering use the same terrain.

```bash
cd /home/user/projects/new_blog/docs/Simulation/Gazebo/demo_worlds/heightmap/code
gz sim --force-version 8 heightmap_world.sdf
```

The official Gazebo Harmonic GUI docs recommend `--force-version 8` when more than one Gazebo version is installed.

## Embedded heightmap world

The important part is the `heightmap_terrain` model. It is not included from Fuel or a separate model folder; it is embedded directly in the world file.

<details>
<summary>Complete world</summary>

```xml
--8<-- "docs/Simulation/Gazebo/demo_worlds/heightmap/code/heightmap_world.sdf"
```
</details>

<details>
<summary>Reusable model file</summary>

```xml
--8<-- "docs/Simulation/Gazebo/demo_worlds/heightmap/code/heightmap_model.sdf"
```
</details>

## Heightmap elements

```xml
<heightmap>
  <uri>media/ridge_heightmap.png</uri>
  <size>60 60 8</size>
  <pos>0 0 0</pos>
  <sampling>2</sampling>
  <texture>
    <size>8</size>
    <diffuse>media/terrain_diffuse.png</diffuse>
    <normal>media/flat_normal.png</normal>
  </texture>
</heightmap>
```

| Element | Meaning |
|---|---|
| `<uri>` | Path to the grayscale image. In this example it is relative to the SDF file. |
| `<size>x y z</size>` | Terrain width, length, and height range in meters. Here the image becomes a 60 m by 60 m terrain with up to 8 m of relief. |
| `<pos>` | Offset for the generated terrain. |
| `<sampling>` | Samples per heightmap datum. Higher values look smoother but cost more. |
| `<texture>` | Diffuse and normal textures used by the visual terrain. |
| `<blend>` | Optional. Use it when you add multiple textures and want different materials by elevation. |
| `<use_terrain_paging>` | Optional. Useful for very large terrains if the renderer supports terrain paging. |

!!! note
    The SDFormat spec describes `<heightmap>` as a geometry based on a 2D grayscale image. It also supports DEM files. Gazebo Rendering's heightmap example shows both image heightmaps and DEM heightmaps.

## Creating the heightmap image

Use a square grayscale image. Power-of-two-plus-one sizes such as `129x129`, `257x257`, or `513x513` are common for terrain grids.

- Black means the lowest terrain height.
- White means the highest terrain height.
- Smooth gradients create slopes.
- Sharp brightness changes create cliffs or collision edges.
- Increase the third value in `<size>` to exaggerate height.

For example, if `<size>60 60 8</size>`:

- black pixels are near `z = 0`
- white pixels are near `z = 8`
- a 50% gray pixel is near `z = 4`

## Alternatives for terrain

Heightmap image:
Best for local test worlds, rough outdoor terrain, and hand-authored hills. This is the simplest option.

DEM heightmap:
Best for real terrain data from GIS sources. Gazebo Harmonic lists DEM support, and the rendering example loads a `.tif` DEM. With DEM data, watch the vertical offset because real elevation files may include negative minimum elevation.

Mesh terrain:
Use `<mesh>` with `.dae`, `.obj`, or `.stl` when you need overhangs, caves, walls, buildings, or artist-authored geometry. Heightmaps cannot represent vertical overhangs because each x-y point has only one height.

SDF image extrusion:
SDFormat also has `<image>` geometry, which extrudes boxes from a grayscale image. It is useful for blocky obstacles, occupancy-map-like layouts, and quick maze or wall tests, not natural terrain.

Primitive terrain:
Use planes, boxes, ramps, and cylinders for deterministic unit tests. This is better than a heightmap when you need exact contact geometry.

Fuel or GUI world editing:
Gazebo can insert models from Fuel and save worlds from the GUI. This is useful for assembling a scene quickly, then you can edit the saved SDF by hand.

## Common issues

- If the robot falls through the terrain, make sure the `<collision>` also uses the heightmap geometry.
- If the terrain appears flat, increase the z value in `<size>`.
- If textures do not load, run Gazebo from the `code` folder or use paths resolvable from `GZ_SIM_RESOURCE_PATH`.
- If startup uses the wrong Gazebo version, run `gz sim --versions` and use `gz sim --force-version 8 ...` for Harmonic.

## References

- [SDFormat geometry spec: heightmap](https://sdformat.org/spec/1.12/geometry/)
- [Gazebo Rendering heightmap example](https://gazebosim.org/api/rendering/8/heightmap.html)
- [Gazebo Harmonic feature comparison](https://gazebosim.org/docs/harmonic/comparison/)
- [Gazebo Harmonic GUI: force version](https://gazebosim.org/docs/harmonic/gui/)
