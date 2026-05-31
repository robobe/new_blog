#!/usr/bin/env python3
import json
import math
import subprocess
import sys
import struct
import zlib
from pathlib import Path
from xml.sax.saxutils import escape


def run_gdalinfo(path: Path) -> dict:
    result = subprocess.run(
        ["gdalinfo", "-json", "-stats", str(path)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return json.loads(result.stdout)


def band_min_max(info: dict) -> tuple[float, float]:
    band = info["bands"][0]
    metadata = band.get("metadata", {})
    stats = metadata.get("", {})

    min_value = stats.get("STATISTICS_MINIMUM", band.get("minimum"))
    max_value = stats.get("STATISTICS_MAXIMUM", band.get("maximum"))

    if min_value is None or max_value is None:
        raise RuntimeError("DEM statistics are missing. Run gdalinfo -stats on the DEM.")

    return float(min_value), float(max_value)


def pixel_size(info: dict) -> tuple[float, float]:
    transform = info["geoTransform"]
    return abs(float(transform[1])), abs(float(transform[5]))


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


def ensure_texture_assets(output_dir: Path) -> None:
    texture_dir = output_dir / "textures"
    texture_dir.mkdir(parents=True, exist_ok=True)

    diffuse = texture_dir / "dem_diffuse.png"
    if not diffuse.exists():
        rows = []
        for y in range(64):
            row = []
            for x in range(64):
                noise = ((x * 31 + y * 17 + (x * y) % 19) % 33) - 16
                row.extend([
                    max(0, min(255, 105 + noise)),
                    max(0, min(255, 120 + noise)),
                    max(0, min(255, 82 + noise // 2)),
                ])
            rows.append(row)
        write_png(diffuse, 64, 64, rows)

    normal = texture_dir / "flat_normal.png"
    if not normal.exists():
        write_png(normal, 8, 8, [[128, 128, 255] * 8 for _ in range(8)])


def world_sdf(dem_path: Path, width_m: float, depth_m: float, z_range: float, z_pos: float) -> str:
    dem_uri = escape(dem_path.name)
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="dem_terrain_demo">
    <gravity>0 0 -9.8</gravity>

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
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

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 1200 0 0 0</pose>
      <diffuse>0.9 0.86 0.78 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.45 0.35 -0.82</direction>
    </light>

    <model name="dem_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>{dem_uri}</uri>
              <size>{width_m:.3f} {depth_m:.3f} {z_range:.3f}</size>
              <pos>0 0 {z_pos:.3f}</pos>
              <sampling>1</sampling>
            </heightmap>
          </geometry>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <heightmap>
              <uri>{dem_uri}</uri>
              <size>{width_m:.3f} {depth_m:.3f} {z_range:.3f}</size>
              <pos>0 0 {z_pos:.3f}</pos>
              <sampling>1</sampling>
              <texture>
                <diffuse>textures/dem_diffuse.png</diffuse>
                <normal>textures/flat_normal.png</normal>
                <size>50</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="origin_marker">
      <static>true</static>
      <pose>0 0 5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0.1 0.1 1</ambient>
            <diffuse>1 0.1 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
'''


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} DEM_TIF OUT_WORLD_SDF", file=sys.stderr)
        return 1

    dem_path = Path(sys.argv[1]).resolve()
    out_path = Path(sys.argv[2]).resolve()

    info = run_gdalinfo(dem_path)
    raster_width, raster_height = info["size"]
    pixel_width, pixel_height = pixel_size(info)
    min_elevation, max_elevation = band_min_max(info)

    width_m = raster_width * pixel_width
    depth_m = raster_height * pixel_height
    z_range = max_elevation - min_elevation
    if not math.isfinite(z_range) or z_range <= 0:
        raise RuntimeError("DEM elevation range must be positive.")

    z_pos = -max_elevation

    ensure_texture_assets(out_path.parent)
    out_path.write_text(
        world_sdf(dem_path, width_m, depth_m, z_range, z_pos),
        encoding="utf-8",
    )

    print(f"Wrote: {out_path}")
    print(f"DEM size: {raster_width} x {raster_height} pixels")
    print(f"World size: {width_m:.3f} x {depth_m:.3f} x {z_range:.3f} m")
    print(f"Elevation: min={min_elevation:.3f} max={max_elevation:.3f}")
    print(f"Terrain pos z: {z_pos:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
