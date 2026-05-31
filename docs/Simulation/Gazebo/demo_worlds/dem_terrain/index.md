---
title: DEM Terrain from OpenTopography
tags:
    - gazebo
    - harmonic
    - terrain
    - dem
    - opentopography
---

# DEM terrain from OpenTopography

This page shows how to download a real Digital Elevation Model (DEM) from OpenTopography and use it as terrain in Gazebo Harmonic.

The default example downloads a `2 km x 2 km` area using `USGS10m`, if the selected area is covered by USGS 3DEP. For a lighter Gazebo world, use the `0.5 km x 0.5 km` option shown below. A 10 m DEM gives about `50 x 50` height samples for a 0.5 km square.

!!! note
    `USGS10m` is a USGS 3DEP dataset, so it covers the United States, Alaska, Hawaii, and U.S. territories. For other countries, use a global dataset such as `COP30`, `NASADEM`, `AW3D30`, or `SRTMGL1`. Those are usually 30 m resolution, not 10 m.

!!! tip "request size"
    A `2 km x 2 km` world is only `4 km2`. OpenTopography documents a much larger request limit for `USGS10m`, so the simulation size here is chosen for Gazebo performance, not because the API requires such a small area.

## Install tools

Gazebo uses GDAL internally for DEM files. Use GDAL on the command line to inspect and reproject the DEM before loading it in simulation.

```bash
sudo apt update
sudo apt install gdal-bin python3-gdal python3-numpy curl
```

Check:

```bash
gdalinfo --version
python3 -c "from osgeo import gdal; print(gdal.VersionInfo())"
gz sim --versions
```

## Get an OpenTopography API key

1. Open [OpenTopography](https://opentopography.org/){:target="_blank" rel="noopener noreferrer"}.
2. Create an account or sign in.
3. Open your MyOpenTopo dashboard.
4. Click `Get an API Key` or `Request API key`.
5. Store it in an environment variable:

```bash
export OPENTOPOGRAPHY_API_KEY="your_key_here"

```

Do not commit the API key into the repository.

OpenTopography also documents daily API limits. Keep generated examples and notebooks using `OPENTOPOGRAPHY_API_KEY` from the environment instead of hard-coding a key.

## Pick an area

Choose the center of the terrain as latitude and longitude.

Example near Mount Baker, Washington:

```bash
export CENTER_LAT=48.7769
export CENTER_LON=-121.8144
```

The download script converts the center point into an approximate square latitude/longitude bounding box. By default it downloads `2000 m x 2000 m`.

[Download script](code/download_usgs10m_2km.sh)

```bash
./download_usgs10m_2km.sh "$CENTER_LAT" "$CENTER_LON"
```

Output:

```text
raw/ot_usgs10m_2000m.tif
```

For a smaller `0.5 km x 0.5 km` area, pass `500` as the third argument:

```bash
./download_usgs10m_2km.sh "$CENTER_LAT" "$CENTER_LON" 500
```

Output:

```text
raw/ot_usgs10m_500m.tif
```

The script calls:

```text
https://portal.opentopography.org/API/usgsdem
```

with:

| Parameter | Value |
|---|---|
| `datasetName` | `USGS10m` |
| `south north west east` | bounding box around the center point |
| `outputFormat` | `GTiff` |
| `API_Key` | from `OPENTOPOGRAPHY_API_KEY` |

## Prepare the DEM for Gazebo

The downloaded DEM is usually in geographic coordinates, meaning latitude and longitude degrees. For simulation, reproject it to a meter-based coordinate system.

The script below chooses the UTM zone from the longitude and resamples to 10 m cells:

[Prepare script](code/prepare_dem_utm.sh)

```bash
./prepare_dem_utm.sh "$CENTER_LAT" "$CENTER_LON"
```

Output:

```text
dem_2000m_10m_utm.tif
dem_2000m_10m_utm.info.txt
textures/dem_diffuse.png
textures/flat_normal.png
```

For the `0.5 km x 0.5 km` DEM, pass the same size:

```bash
./prepare_dem_utm.sh "$CENTER_LAT" "$CENTER_LON" 500
```

Output:

```text
dem_500m_10m_utm.tif
dem_500m_10m_utm.info.txt
```

Inspect the file:

```bash
gdalinfo -stats dem_500m_10m_utm.tif
```

Look for:

- `Size is ...`
- `Pixel Size = (10.000..., -10.000...)`
- `Minimum=...`
- `Maximum=...`

For a `2 km x 2 km` area at 10 m resolution, expect roughly `200 x 200` pixels. For a `0.5 km x 0.5 km` area at 10 m resolution, expect roughly `50 x 50` pixels. It may not be exact because the first crop is defined in latitude/longitude.

## Generate the Gazebo world

Generate the SDF from the actual DEM metadata:

[World generator](code/make_dem_world.py)

```bash
./make_dem_world.py dem_500m_10m_utm.tif dem_world.sdf
```

Then run:

```bash
gz sim -r -v4 dem_world.sdf
```

The generated SDF uses the DEM as both collision and visual geometry:

```xml
<heightmap>
  <uri>dem_500m_10m_utm.tif</uri>
  <size>500 500 120</size>
  <pos>0 0 -650</pos>
  <sampling>1</sampling>
  <texture>
    <diffuse>textures/dem_diffuse.png</diffuse>
    <normal>textures/flat_normal.png</normal>
    <size>50</size>
  </texture>
</heightmap>
```

The exact `size` and `pos` values are generated from `gdalinfo`:

- `x = raster_width_pixels * pixel_width_m`
- `y = raster_height_pixels * pixel_height_m`
- `z = Maximum elevation - Minimum elevation`
- `pos z = -Maximum elevation`

Using `pos z = -Maximum elevation` moves the highest point near `z = 0`, which makes the terrain easier to view from the default Gazebo camera. If you want the lowest point near `z = 0`, change `pos z` to `-Minimum elevation`.

## Fuel-style mesh model

The Gazebo moon examples show two useful patterns:

- `dem_moon.sdf` is a world that configures moon gravity, GUI, physics, a falling box, and then includes a terrain model.
- The `Moon 60S 30S` Fuel model is a regular model folder with `model.config`, `model.sdf`, and a `.dae` mesh. It is not loaded as a DEM heightmap at runtime.

This mesh approach can be more stable for rendering because Gazebo uses the normal mesh pipeline instead of the OGRE heightmap terrain shader.

[Mesh model generator](code/dem_to_mesh_model.py)

```bash
./dem_to_mesh_model.py dem_500m_10m_utm.tif .
```

Output:

```text
models/local_dem_terrain/model.config
models/local_dem_terrain/model.sdf
models/local_dem_terrain/meshes/dem_terrain.dae
dem_mesh_world.sdf
```

The generated model is structured like a Fuel model:

```text
local_dem_terrain/
├── model.config
├── model.sdf
└── meshes/
    └── dem_terrain.dae
```

The world includes it with a model URI:

```xml
<include>
  <uri>model://local_dem_terrain</uri>
</include>
```

Before running the world, point Gazebo at the local model folder:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/models"
gz sim -r -v4 dem_mesh_world.sdf
```

The mesh generator converts the DEM grid into a centered terrain mesh:

- `x` and `y` are local meter coordinates from the UTM raster.
- `z = elevation - minimum_elevation`, so the lowest point is near `z = 0`.
- The generated `.dae` is used for both collision and visual geometry.
- The world uses DART with Bullet collision detection, matching the moon DEM example's heightmap-friendly physics setting.

## Files

<details>
<summary>Download script</summary>

```bash
--8<-- "docs/Simulation/Gazebo/demo_worlds/dem_terrain/code/download_usgs10m_2km.sh"
```
</details>

<details>
<summary>Prepare script</summary>

```bash
--8<-- "docs/Simulation/Gazebo/demo_worlds/dem_terrain/code/prepare_dem_utm.sh"
```
</details>

<details>
<summary>Mesh model generator</summary>

```python
--8<-- "docs/Simulation/Gazebo/demo_worlds/dem_terrain/code/dem_to_mesh_model.py"
```
</details>

<details>
<summary>World generator</summary>

```python
--8<-- "docs/Simulation/Gazebo/demo_worlds/dem_terrain/code/make_dem_world.py"
```
</details>

## If 10 m is not available

If OpenTopography returns an error for `USGS10m`, the selected point is probably outside USGS 3DEP coverage or the account does not have access.

For non-U.S. terrain, use a 30 m global dataset. Example API endpoint:

```text
https://portal.opentopography.org/API/globaldem?demtype=COP30&south=...&north=...&west=...&east=...&outputFormat=GTiff&API_Key=...
```

For a `0.5 km x 0.5 km` world, 30 m DEM gives about `17 x 17` samples. For a `2 km x 2 km` world, it gives about `67 x 67` samples. It is still useful for large terrain shape, but it will not show small local features.

## Common problems

- If Gazebo cannot load the `.tif`, run `gdalinfo dem_500m_10m_utm.tif` first. If GDAL cannot read it, Gazebo will not read it.
- If the terrain is huge or tiny, check that the DEM was reprojected to UTM meters before generating the world.
- If the terrain is visually offset from collision, make sure the same `<heightmap>` values are used in both `<collision>` and `<visual>`.
- If objects do not fall, make sure simulation is running. Use `gz sim -r -v4 dem_world.sdf` or press the play button in the GUI. Also check that the inserted model does not have `<static>true</static>`.
- If OGRE crashes with a fragment shader error like `detailCol0 undeclared`, make sure the visual heightmap has a `<texture>` block with valid diffuse and normal texture files.
- If simulation is slow, resample to 20 m or 30 m with `gdalwarp -tr 20 20`.
- If the terrain is hard to find in the GUI, check the generated `<pos>` z value.
- If `dem_mesh_world.sdf` cannot find `model://local_dem_terrain`, set `GZ_SIM_RESOURCE_PATH="$PWD/models"` from the folder that contains the generated `models` directory.

## References

- [Gazebo Sim DEM heightmap tutorial](https://gazebosim.org/api/sim/10/heightmap_dem.html){:target="_blank" rel="noopener noreferrer"}
- [Gazebo `dem_moon.sdf` example](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/dem_moon.sdf){:target="_blank" rel="noopener noreferrer"}
- [SDFormat heightmap spec](https://sdformat.org/spec/1.12/geometry/){:target="_blank" rel="noopener noreferrer"}
- [Moon 60S 30S Fuel model](https://app.gazebosim.org/jasmeetsingh/fuel/models/Moon%2060S%2030S){:target="_blank" rel="noopener noreferrer"}
- [OpenTopography API access to USGS 3DEP rasters](https://opentopography.org/news/api-access-usgs-3dep-rasters-now-available){:target="_blank" rel="noopener noreferrer"}
- [OpenTopography API developer notes](https://opentopography.org/developers){:target="_blank" rel="noopener noreferrer"}
