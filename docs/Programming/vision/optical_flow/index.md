---
tags:
    - opencv
    - optical flow
    - gimbal
---


{{ page_folder_links() }}


```python
#!/usr/bin/env python3
import numpy as np
import cv2

# ---------- Utilities ----------
def quat_to_R(q):
    """
    q = [w, x, y, z] (Hamilton, normalized). Returns 3x3 rotation matrix.
    Rotation maps world->camera if q is that orientation; adapt to your convention.
    """
    w, x, y, z = q
    # Normalize (defensive)
    n = np.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        raise ValueError("Zero-norm quaternion")
    w, x, y, z = w/n, x/n, y/n, z/n
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=float)
    return R

def derotate_prev_frame(img_prev, K, rvec):
    """
    rvec: Rodrigues rotation vector (rx, ry, rz) mapping prev->curr
          in the CAMERA OPTICAL frame (x right, y down, z forward).
    """
    R, _ = cv2.Rodrigues(rvec)
    H = K @ R @ np.linalg.inv(K)
    h, w = img_prev.shape[:2]
    return cv2.warpPerspective(img_prev, H, (w, h),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT)

# ---------- Example main ----------
if __name__ == "__main__":
    # --- Load two consecutive frames (grayscale) ---
    # Replace with your image paths or camera grab
    img_prev = cv2.imread("frame_prev.png", cv2.IMREAD_GRAYSCALE)
    img_curr = cv2.imread("frame_curr.png", cv2.IMREAD_GRAYSCALE)
    if img_prev is None or img_curr is None:
        raise SystemExit("Could not load frame_prev.png or frame_curr.png")

    # --- Camera intrinsics (after undistortion/rectification if you use it) ---
    # Replace with your calibrated values
    fx, fy = 700.0, 700.0
    cx, cy = img_prev.shape[1] / 2.0, img_prev.shape[0] / 2.0
    K = np.array([[fx, 0,  cx],
                  [0,  fy, cy],
                  [0,   0,  1]], dtype=float)

    # If your lens is distorted, undistort both frames first, then use the *new* K:
    # dist = np.array([k1, k2, p1, p2, k3], dtype=float)
    # img_prev = cv2.undistort(img_prev, K, dist)
    # img_curr = cv2.undistort(img_curr, K, dist)
    # (If you use getOptimalNewCameraMatrix, update K accordingly.)

    # --- Build rotation prev->curr (choose ONE of the two methods below) ---

    # (A) From gyro angular velocity in camera frame (rad/s) and dt (s)
    use_gyro = True
    if use_gyro:
        # Replace with your synchronized measurements
        wx, wy, wz = 0.02, -0.01, 0.00     # rad/s (camera optical frame)
        dt = 0.033                         # 33 ms between frames
        rvec = np.array([wx, wy, wz], dtype=float) * dt  # small-angle approx

    else:
        # (B) From two quaternions (camera orientation at t-1 and t)
        # q = [w, x, y, z] in same frame convention both times
        q_prev = np.array([0.9998, 0.0, 0.0175, 0.0])  # example
        q_curr = np.array([0.9997, 0.0, 0.0245, 0.0])  # example
        R_prev = quat_to_R(q_prev)
        R_curr = quat_to_R(q_curr)
        # Rotation mapping prev cam to curr cam:
        R = R_curr @ R_prev.T
        # Convert R to Rodrigues rvec:
        rvec, _ = cv2.Rodrigues(R)

    # --- Derotate previous frame to current orientation ---
    img_prev_derot = derotate_prev_frame(img_prev, K, rvec)

    # --- Optical flow on derotated previous vs current ---
    flow = cv2.calcOpticalFlowFarneback(
        img_prev_derot, img_curr, None,
        pyr_scale=0.5, levels=3, winsize=21,
        iterations=3, poly_n=5, poly_sigma=1.2, flags=0
    )
    u = flow[..., 0]
    v = flow[..., 1]

    # Robust summary of residual flow (should be near zero if rotation-only)
    u_med = float(np.median(u))
    v_med = float(np.median(v))
    print(f"Residual flow median (u, v) = ({u_med:.3f}, {v_med:.3f}) px/frame")

    # --- (Optional) visualize flow magnitude ---
    mag = np.sqrt(u*u + v*v)
    mag_norm = np.clip((mag / (mag.max() + 1e-6)) * 255.0, 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(mag_norm, cv2.COLORMAP_TURBO)
    overlay = cv2.addWeighted(cv2.cvtColor(img_curr, cv2.COLOR_GRAY2BGR), 0.6, color, 0.4, 0)

    cv2.imshow("prev (raw)", img_prev)
    cv2.imshow("prev derotated", img_prev_derot)
    cv2.imshow("curr", img_curr)
    cv2.imshow("flow magnitude overlay", overlay)
    print("Press any key to exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

```