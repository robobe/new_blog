import h5py

def print_hdf5_structure(g, indent=0):
    """Recursively print the structure of an HDF5 file/group."""
    spacing = "  " * indent
    if isinstance(g, h5py.Dataset):
        print(f"{spacing}- Dataset: {g.name}, shape={g.shape}, dtype={g.dtype}")
    elif isinstance(g, h5py.Group):
        print(f"{spacing}+ Group: {g.name}")
        for key in g.keys():
            print_hdf5_structure(g[key], indent + 1)

# Open MVSEC file (replace with your file path)
file_path = "/home/user/Downloads/outdoor_day1_gt-001.hdf5"
with h5py.File(file_path, "r") as f:
    print("Top-level groups:", list(f.keys()))
    print("\nFull structure:\n")
    print_hdf5_structure(f)
