#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 CENTER_LAT CENTER_LON [SIZE_M]" >&2
    echo "Example 500 m square: $0 48.7769 -121.8144 500" >&2
    exit 1
fi

if [ -z "${OPENTOPOGRAPHY_API_KEY:-}" ]; then
    echo "Set OPENTOPOGRAPHY_API_KEY before running this script." >&2
    exit 1
fi

CENTER_LAT="$1"
CENTER_LON="$2"
SIZE_M="${3:-2000}"
HALF_SIZE_M="$(
python3 - "$SIZE_M" <<'PY'
import sys

size_m = float(sys.argv[1])
if size_m <= 0:
    raise SystemExit("SIZE_M must be positive.")
print(size_m / 2.0)
PY
)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${SCRIPT_DIR}/raw"
SIZE_LABEL="$(python3 - "$SIZE_M" <<'PY'
import sys

size_m = float(sys.argv[1])
if size_m.is_integer():
    print(str(int(size_m)))
else:
    print(str(size_m).replace(".", "p"))
PY
)"
OUT_FILE="${OUT_DIR}/ot_usgs10m_${SIZE_LABEL}m.tif"

mkdir -p "$OUT_DIR"

read -r SOUTH NORTH WEST EAST < <(
python3 - "$CENTER_LAT" "$CENTER_LON" "$HALF_SIZE_M" <<'PY'
import math
import sys

lat = float(sys.argv[1])
lon = float(sys.argv[2])
half_size_m = float(sys.argv[3])

meters_per_degree_lat = 111_320.0
meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(lat))

if abs(meters_per_degree_lon) < 1:
    raise SystemExit("Longitude scale is too small near the pole.")

dlat = half_size_m / meters_per_degree_lat
dlon = half_size_m / meters_per_degree_lon

print(f"{lat - dlat:.8f} {lat + dlat:.8f} {lon - dlon:.8f} {lon + dlon:.8f}")
PY
)

URL="https://portal.opentopography.org/API/usgsdem"

curl --fail --location --get "$URL" \
    --data-urlencode "datasetName=USGS10m" \
    --data-urlencode "south=${SOUTH}" \
    --data-urlencode "north=${NORTH}" \
    --data-urlencode "west=${WEST}" \
    --data-urlencode "east=${EAST}" \
    --data-urlencode "outputFormat=GTiff" \
    --data-urlencode "API_Key=${OPENTOPOGRAPHY_API_KEY}" \
    --output "$OUT_FILE"

echo "Downloaded: $OUT_FILE"
gdalinfo "$OUT_FILE" | sed -n '1,35p'
