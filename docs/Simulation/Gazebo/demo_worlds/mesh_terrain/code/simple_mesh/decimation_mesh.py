import trimesh

src = "models/dem_terrain/meshes/example_dem_100m.dae"
dst = "models/dem_terrain/meshes/example_dem_100m_collision.dae"

mesh = trimesh.load(src, force="mesh")

# This assumes the terrain is a regular grid.
# Current mesh has 676 vertices = 26 x 26 grid.
grid_size = 26
step = 2  # keep every second point -> about 13 x 13 grid

vertices = mesh.vertices.reshape((grid_size, grid_size, 3))
reduced_vertices = vertices[::step, ::step]

rows, cols, _ = reduced_vertices.shape
flat_vertices = reduced_vertices.reshape((-1, 3))

faces = []
for y in range(rows - 1):
    for x in range(cols - 1):
        i0 = y * cols + x
        i1 = y * cols + x + 1
        i2 = (y + 1) * cols + x
        i3 = (y + 1) * cols + x + 1

        faces.append([i0, i2, i1])
        faces.append([i1, i2, i3])

collision_mesh = trimesh.Trimesh(
    vertices=flat_vertices,
    faces=faces,
    process=False,
)

collision_mesh.export(dst)

print("saved:", dst)
print("vertices:", len(collision_mesh.vertices))
print("faces:", len(collision_mesh.faces))