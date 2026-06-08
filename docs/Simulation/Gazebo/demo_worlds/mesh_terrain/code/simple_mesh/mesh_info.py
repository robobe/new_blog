import trimesh

mesh = trimesh.load("models/dem_terrain/meshes/example_dem_100m.dae", force="mesh")

print("vertices:", len(mesh.vertices))
print("faces:", len(mesh.faces))
print("bounds:", mesh.bounds)
print("size xyz:", mesh.extents)