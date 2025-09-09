import h5py
import cv2
import numpy as np

# Path to your MVSEC dataset file (data file, not gt)
file_path = "/home/user/Downloads/outdoor_day1_data-004.hdf5"

# Open HDF5 file
with h5py.File(file_path, "r") as f:
    # Explore groups
    print("Top-level groups:", list(f.keys()))
    
    # APS frames are typically under: /davis/left/images/data
    images = f["davis"]["left"]["image_raw"]
    
    
    print("Number of images:", images.shape[0])
    print("Image shape:", images.shape[1:])  # (height, width)
    
    # Extract the first image
    first_img = images[0]  # shape (H, W), dtype=uint8
    
    # Show with OpenCV
    cv2.imshow("First APS Image", first_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
