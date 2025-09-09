from math import radians, degrees, sin, cos, atan2, asin

def move_point(lat, lon, distance_m, bearing_deg):
    R = 6371000  # Earth radius in meters

    lat1 = radians(lat)
    lon1 = radians(lon)
    bearing = radians(bearing_deg)

    lat2 = asin(sin(lat1) * cos(distance_m/R) +
                cos(lat1) * sin(distance_m/R) * cos(bearing))

    lon2 = lon1 + atan2(sin(bearing) * sin(distance_m/R) * cos(lat1),
                        cos(distance_m/R) - sin(lat1) * sin(lat2))

    return degrees(lat2), degrees(lon2)

# Example: move 500m east
lat, lon = 32.0853, 34.7818
new_lat, new_lon = move_point(lat, lon, 500, 90)

print(f"Original: {lat:.6f}, {lon:.6f}")
print(f"New point: {new_lat:.6f}, {new_lon:.6f}")