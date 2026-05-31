#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 CENTER_LAT CENTER_LON [SIZE_M]" >&2
    echo "Example 500 m square: $0 48.7769 -121.8144 500" >&2
    exit 1
fi

CENTER_LAT="$1"
CENTER_LON="$2"
SIZE_M="${3:-2000}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIZE_LABEL="$(python3 - "$SIZE_M" <<'PY'
import sys

size_m = float(sys.argv[1])
if size_m.is_integer():
    print(str(int(size_m)))
else:
    print(str(size_m).replace(".", "p"))
PY
)"
IN_FILE="${SCRIPT_DIR}/raw/ot_usgs10m_${SIZE_LABEL}m.tif"
OUT_FILE="${SCRIPT_DIR}/dem_${SIZE_LABEL}m_10m_utm.tif"

if [ ! -f "$IN_FILE" ]; then
    echo "Missing $IN_FILE. Run download_usgs10m_2km.sh with the same SIZE_M first." >&2
    exit 1
fi

UTM_EPSG="$(
python3 - "$CENTER_LAT" "$CENTER_LON" <<'PY'
import math
import sys

lat = float(sys.argv[1])
lon = float(sys.argv[2])
zone = int(math.floor((lon + 180.0) / 6.0) + 1)
epsg = 32600 + zone if lat >= 0 else 32700 + zone
print(epsg)
PY
)"

gdalwarp \
    -overwrite \
    -t_srs "EPSG:${UTM_EPSG}" \
    -tr 10 10 \
    -r bilinear \
    -of GTiff \
    -co COMPRESS=DEFLATE \
    "$IN_FILE" \
    "$OUT_FILE"

gdalinfo -stats "$OUT_FILE" > "${OUT_FILE%.tif}.info.txt"

echo "Prepared: $OUT_FILE"
echo "Projection: EPSG:${UTM_EPSG}"
sed -n '1,80p' "${OUT_FILE%.tif}.info.txt"
