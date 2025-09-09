from math import radians, sin, cos, sqrt, atan2, degrees

def haversine(coord1, coord2):
    # Coordinates: (lat, lon)
    R = 6371.0  # Earth radius in kilometers

    lat1, lon1 = map(radians, coord1)
    lat2, lon2 = map(radians, coord2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))

    return R * c

def bearing_between_points(lat1, lon1, lat2, lon2):
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1

    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)

    initial_bearing = atan2(x, y)
    
    # Convert from radians to degrees and normalize 0-360
    bearing = (degrees(initial_bearing) + 360) % 360
    return bearing

# Example
origin = (32.0853, 34.7818)   # Tel Aviv
target = (32.085300, 34.787107)   # next point ~500m east

print(f"Distance: {haversine(origin, target):.2f} km")
print(f"Bearing (from north cw): {bearing_between_points(32.0853, 34.7818, 32.085300, 34.787107):.2f}°")
