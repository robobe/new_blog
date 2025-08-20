import numpy as np

def make_checkerboard(h, w, cell=16):
    """Binary checkerboard: 0/255, square size = cell px."""
    yy, xx = np.indices((h, w))
    board = ((xx // cell + yy // cell) % 2) * 255
    return board.astype(np.uint8)

def add_axis_aligned_lines(img, step=40, thickness=2, value=255):
    """Add a light grid of vertical/horizontal lines to create extra corners."""
    h, w = img.shape
    out = img.copy()
    half = thickness // 2
    for x in range(step//2, w, step):
        x0 = max(0, x - half); x1 = min(w, x + half + 1)
        out[:, x0:x1] = value
    for y in range(step//2, h, step):
        y0 = max(0, y - half); y1 = min(h, y + half + 1)
        out[y0:y1, :] = value
    return out

def add_dot_grid_numpy(img, spacing=24, radius=3, value=255):
    """
    Dot grid without OpenCV: draws filled circles using NumPy masks.
    (Fast enough for typical 640x480 with ~500 dots.)
    """
    h, w = img.shape
    Y, X = np.ogrid[:h, :w]
    out = img.copy()
    r2 = radius*radius
    for cy in range(spacing//2, h, spacing):
        for cx in range(spacing//2, w, spacing):
            mask = (X - cx)**2 + (Y - cy)**2 <= r2
            out[mask] = value
    return out

def add_noise_and_clip(img, sigma=5.0):
    """Tiny Gaussian-ish noise to avoid perfectly flat regions."""
    if sigma <= 0:
        return img
    rng = np.random.default_rng(0)
    noise = rng.normal(0, sigma, size=img.shape).astype(np.float32)
    out = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return out

def make_harris_test_image(h=480, w=640,
                           cell=16, line_step=48, line_thickness=2,
                           dot_spacing=24, dot_radius=3, noise_sigma=4.0):
    """
    Returns a uint8 grayscale image with rich, corner-friendly structure.
    """
    img = make_checkerboard(h, w, cell=cell)
    img = add_axis_aligned_lines(img, step=line_step, thickness=line_thickness, value=255)
    img = add_dot_grid_numpy(img, spacing=dot_spacing, radius=dot_radius, value=255)
    img = add_noise_and_clip(img, sigma=noise_sigma)
    return img

# Example usage + quick Harris demo (OpenCV optional)
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import vpi
    img = make_harris_test_image()
    # Convert grayscale image to BGR for VPI
    img = np.stack([img]*3, axis=-1)
    print("Harris test image shape:", img.shape)
    with vpi.Backend.CUDA:
        frame = vpi.asimage(img, vpi.Format.BGR8).convert(vpi.Format.U8)
        curFeatures, scores = frame.harriscorners(strength=0.1, sensitivity=0.01)
        print(curFeatures.type)

    scores_np = scores.cpu()
    print(type(scores_np), scores_np.shape, scores_np.dtype)
    exit()
    # Show the generated image
    plt.figure(figsize=(6,4))
    plt.imshow(img, cmap="gray", vmin=0, vmax=255)
    plt.title("Synthetic Harris-friendly image"); plt.axis("off")
    plt.show()

    # # Optional: run Harris with OpenCV if available
    # try:
    #     import cv2
    #     gray32 = np.float32(img)
    #     # blockSize=2..5, ksize must be odd (3,5,7), k≈0.04–0.06
    #     dst = cv2.cornerHarris(gray32, blockSize=3, ksize=3, k=0.04)
    #     dst = (dst - dst.min()) / (dst.max() - dst.min() + 1e-8)  # normalize for display
    #     overlay = np.dstack([img, img, img]).astype(np.float32)
    #     mask = dst > 0.3
    #     overlay[mask] = [255, 0, 0]  # mark corners in red

    #     plt.figure(figsize=(6,4))
    #     plt.imshow(overlay.astype(np.uint8))
    #     plt.title("Harris corners (red)"); plt.axis("off")
    #     plt.show()
    # except ImportError:
    #     print("OpenCV not installed; skipping Harris demo. The image is ready for any detector.")
