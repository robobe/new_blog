#!/usr/bin/python3
import math
import sys
import struct
import zlib
from pathlib import Path
from xml.sax.saxutils import escape

try:
    import numpy as np
    from osgeo import gdal
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing Python dependency. Install it on Ubuntu with:\n"
        "  sudo apt install python3-gdal python3-numpy\n"
        "If you use a virtual environment, install GDAL bindings that match your system GDAL."
    ) from exc

gdal.UseExceptions()


def load_dem(path: Path) -> tuple[np.ndarray, float, float, float, float, float, float]:
    dataset = gdal.Open(str(path))
    if dataset is None:
        raise RuntimeError(f"Could not open DEM: {path}")

    band = dataset.GetRasterBand(1)
    data = band.ReadAsArray().astype(float)
    nodata = band.GetNoDataValue()
    valid = np.isfinite(data)
    if nodata is not None:
        valid &= data != nodata

    if not np.any(valid):
        raise RuntimeError("DEM has no valid elevation samples.")

    min_elevation = float(np.min(data[valid]))
    max_elevation = float(np.max(data[valid]))
    data[~valid] = min_elevation

    transform = dataset.GetGeoTransform()
    pixel_width = abs(float(transform[1]))
    pixel_height = abs(float(transform[5]))
    width_m = data.shape[1] * pixel_width
    depth_m = data.shape[0] * pixel_height

    return data, pixel_width, pixel_height, width_m, depth_m, min_elevation, max_elevation


def normal_for_triangle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    normal = np.cross(b - a, c - a)
    length = np.linalg.norm(normal)
    if length == 0:
        return np.array([0.0, 0.0, 1.0])
    return normal / length


def vertex_normals(vertices: np.ndarray, triangles: list[tuple[int, int, int]]) -> np.ndarray:
    normals = np.zeros_like(vertices)
    for i0, i1, i2 in triangles:
        normal = normal_for_triangle(vertices[i0], vertices[i1], vertices[i2])
        normals[i0] += normal
        normals[i1] += normal
        normals[i2] += normal

    lengths = np.linalg.norm(normals, axis=1)
    zero = lengths == 0
    lengths[zero] = 1.0
    normals = normals / lengths[:, None]
    normals[zero] = np.array([0.0, 0.0, 1.0])
    return normals


def write_png(path: Path, width: int, height: int, rows: list[list[int]]) -> None:
    raw = b"".join(b"\x00" + bytes(row) for row in rows)

    def chunk(kind: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(kind + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", crc)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


def terrain_color(value: float) -> tuple[int, int, int]:
    # A compact elevation color ramp: low green, mid tan, high light rock.
    stops = [
        (0.00, (62, 86, 55)),
        (0.35, (97, 122, 72)),
        (0.65, (146, 132, 92)),
        (1.00, (218, 214, 190)),
    ]
    for (left_pos, left), (right_pos, right) in zip(stops, stops[1:]):
        if value <= right_pos:
            t = (value - left_pos) / (right_pos - left_pos)
            return tuple(round(left[i] + t * (right[i] - left[i])) for i in range(3))
    return stops[-1][1]


def write_texture(path: Path, elevation: np.ndarray, min_elevation: float, max_elevation: float) -> None:
    span = max(max_elevation - min_elevation, 1.0)
    normalized = np.clip((elevation - min_elevation) / span, 0.0, 1.0)
    rows = []
    for row in normalized:
        png_row = []
        for value in row:
            png_row.extend(terrain_color(float(value)))
        rows.append(png_row)
    write_png(path, elevation.shape[1], elevation.shape[0], rows)


def write_dae(
    path: Path,
    vertices: np.ndarray,
    texcoords: np.ndarray,
    triangles: list[tuple[int, int, int]],
    texture_file: str,
) -> None:
    normals = vertex_normals(vertices, triangles)
    position_values = " ".join(f"{value:.6f}" for value in vertices.reshape(-1))
    normal_values = " ".join(f"{value:.6f}" for value in normals.reshape(-1))
    texcoord_values = " ".join(f"{value:.6f}" for value in texcoords.reshape(-1))

    # Each triangle uses the same index for VERTEX, NORMAL, and TEXCOORD.
    triangle_indices = " ".join(
        f"{i0} {i0} {i0} {i1} {i1} {i1} {i2} {i2} {i2}" for i0, i1, i2 in triangles
    )

    path.write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>dem_to_mesh_model.py</authoring_tool>
    </contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>

  <library_effects>
    <effect id="terrain_effect">
      <profile_COMMON>
        <newparam sid="terrain_texture_surface">
          <surface type="2D">
            <init_from>terrain_texture_image</init_from>
          </surface>
        </newparam>
        <newparam sid="terrain_texture_sampler">
          <sampler2D>
            <source>terrain_texture_surface</source>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <lambert>
            <diffuse>
              <texture texture="terrain_texture_sampler" texcoord="UVMap"/>
            </diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>

  <library_images>
    <image id="terrain_texture_image" name="terrain_texture_image">
      <init_from>{escape(texture_file)}</init_from>
    </image>
  </library_images>

  <library_materials>
    <material id="terrain_material" name="terrain_material">
      <instance_effect url="#terrain_effect"/>
    </material>
  </library_materials>

  <library_geometries>
    <geometry id="dem_terrain_mesh" name="dem_terrain_mesh">
      <mesh>
        <source id="dem_terrain_positions">
          <float_array id="dem_terrain_positions_array" count="{vertices.size}">{position_values}</float_array>
          <technique_common>
            <accessor source="#dem_terrain_positions_array" count="{len(vertices)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="dem_terrain_normals">
          <float_array id="dem_terrain_normals_array" count="{normals.size}">{normal_values}</float_array>
          <technique_common>
            <accessor source="#dem_terrain_normals_array" count="{len(normals)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="dem_terrain_uvmap">
          <float_array id="dem_terrain_uvmap_array" count="{texcoords.size}">{texcoord_values}</float_array>
          <technique_common>
            <accessor source="#dem_terrain_uvmap_array" count="{len(texcoords)}" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="dem_terrain_vertices">
          <input semantic="POSITION" source="#dem_terrain_positions"/>
        </vertices>
        <triangles material="terrain_material" count="{len(triangles)}">
          <input semantic="VERTEX" source="#dem_terrain_vertices" offset="0"/>
          <input semantic="NORMAL" source="#dem_terrain_normals" offset="1"/>
          <input semantic="TEXCOORD" source="#dem_terrain_uvmap" offset="2" set="0"/>
          <p>{triangle_indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>

  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="dem_terrain" name="dem_terrain">
        <instance_geometry url="#dem_terrain_mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="terrain_material" target="#terrain_material">
                <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>

  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
''', encoding="utf-8")


def write_model_config(path: Path) -> None:
    path.write_text('''<?xml version="1.0" ?>
<model>
  <name>Local DEM Terrain Mesh</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>

  <author>
    <name>new_blog demo</name>
  </author>

  <description>
    Mesh terrain generated from a local DEM GeoTIFF.
  </description>
</model>
''', encoding="utf-8")


def write_model_sdf(path: Path) -> None:
    path.write_text('''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="local_dem_terrain_mesh">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>meshes/dem_terrain.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <cast_shadows>true</cast_shadows>
        <geometry>
          <mesh>
            <uri>meshes/dem_terrain.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
''', encoding="utf-8")


def write_world(path: Path, width_m: float, depth_m: float, max_z: float) -> None:
    camera_x = -0.65 * width_m
    camera_y = -0.75 * depth_m
    camera_z = max(max_z + 0.9 * max(width_m, depth_m), 500.0)
    box_z = max_z + 50.0
    path.write_text(f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="dem_mesh_world">
    <gravity>0 0 -9.8</gravity>

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.35 0.35 0.35</ambient_light>
        <background_color>0.78 0.82 0.86</background_color>
        <camera_pose>{camera_x:.3f} {camera_y:.3f} {camera_z:.3f} 0 0.95 0.72</camera_pose>
      </plugin>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 1200 0 0 0</pose>
      <diffuse>0.9 0.86 0.78 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.45 0.35 -0.82</direction>
    </light>

    <include>
      <uri>model://local_dem_terrain</uri>
    </include>

    <model name="falling_box">
      <pose>0 0 {box_z:.3f} 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.16666</ixx>
            <iyy>0.16666</iyy>
            <izz>0.16666</izz>
          </inertia>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>10 10 10</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>10 10 10</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0.1 0.1 1</ambient>
            <diffuse>1 0.1 0.1 1</diffuse>
            <specular>0.4 0.4 0.4 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
''', encoding="utf-8")


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} DEM_TIF OUT_CODE_DIR", file=sys.stderr)
        return 1

    dem_path = Path(sys.argv[1]).resolve()
    out_dir = Path(sys.argv[2]).resolve()
    model_dir = out_dir / "models" / "local_dem_terrain"
    mesh_dir = model_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    data, pixel_width, pixel_height, width_m, depth_m, min_elevation, max_elevation = load_dem(dem_path)
    rows, cols = data.shape
    z = data - min_elevation
    max_z = max_elevation - min_elevation

    vertices = []
    texcoords = []
    for row in range(rows):
        y = depth_m / 2.0 - row * pixel_height
        for col in range(cols):
            x = col * pixel_width - width_m / 2.0
            vertices.append((x, y, z[row, col]))
            u = col / (cols - 1) if cols > 1 else 0.0
            v = 1.0 - row / (rows - 1) if rows > 1 else 0.0
            texcoords.append((u, v))
    vertices_np = np.array(vertices, dtype=float)
    texcoords_np = np.array(texcoords, dtype=float)

    triangles = []
    for row in range(rows - 1):
        for col in range(cols - 1):
            i00 = row * cols + col
            i10 = row * cols + col + 1
            i01 = (row + 1) * cols + col
            i11 = (row + 1) * cols + col + 1
            triangles.append((i00, i01, i10))
            triangles.append((i10, i01, i11))

    texture_file = "dem_texture.png"
    write_texture(mesh_dir / texture_file, data, min_elevation, max_elevation)
    write_dae(mesh_dir / "dem_terrain.dae", vertices_np, texcoords_np, triangles, texture_file)
    write_model_config(model_dir / "model.config")
    write_model_sdf(model_dir / "model.sdf")
    write_world(out_dir / "dem_mesh_world.sdf", width_m, depth_m, max_z)

    print(f"Model written: {model_dir}")
    print(f"World written: {out_dir / 'dem_mesh_world.sdf'}")
    print(f"DEM samples: {cols} x {rows}")
    print(f"Mesh vertices: {len(vertices_np)}")
    print(f"Mesh triangles: {len(triangles)}")
    print(f"World size: {width_m:.3f} x {depth_m:.3f} x {max_z:.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
