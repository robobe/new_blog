import numpy as np

def intrinsics_from_hfov(width, height, hfov_deg):
    """
    Build camera intrinsics K from horizontal FOV (degrees) and image size.
    Coordinate convention: x right, y down, z forward.
    """
    hfov = np.deg2rad(hfov_deg)
    fx = (width / 2) / np.tan(hfov / 2)

    # derive vertical FOV from aspect ratio, then fy
    vfov = 2 * np.arctan((height / width) * np.tan(hfov / 2))
    fy = (height / 2) / np.tan(vfov / 2)

    cx, cy = width / 2.0, height / 2.0
    K = np.array([[fx, 0,  cx],
                  [0,  fy, cy],
                  [0,  0,  1]])
    return K

def project_points_cam_to_pixel(P_cam, K):
    """
    Project 3D points in the CAMERA frame (x right, y down, z forward) onto the image.
    P_cam: (N,3) array of [X,Y,Z] with Z>0
    Returns: (N,2) pixel coords (u,v)
    """
    P = np.atleast_2d(P_cam).astype(float)
    X, Y, Z = P[:,0], P[:,1], P[:,2]
    if np.any(Z <= 0):
        raise ValueError("All points must have Z>0 (in front of camera).")

    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    u = fx * (X / Z) + cx
    v = fy * (Y / Z) + cy
    return np.stack([u, v], axis=1)

# --- Example usage ---
W, H = 640, 480
K = intrinsics_from_hfov(W, H, hfov_deg=50.0)

# A point straight in front of the camera center at 3 meters
p_center = np.array([0.0, 0.0, 3.0])
print("Center ->", project_points_cam_to_pixel(p_center, K))  # ~[320, 240]

# A point 0.5 m to the right, 3 m forward
p_right = np.array([0.5, 0.0, 3.0])
print("Right  ->", project_points_cam_to_pixel(p_right, K))

# A point 0.5 m up (remember y is DOWN, so use negative y), 3 m forward
p_up = np.array([-0.5, 0.0, 3.0])
print("Up     ->", project_points_cam_to_pixel(p_up, K))
