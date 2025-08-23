import cv2 as cv
import numpy as np
from pathlib import Path

def create_test_sequence():
    img1 = np.zeros((400, 400, 3), dtype=np.uint8)
    cv.circle(img1, (150, 200), 30, (255, 255, 255), -1)
    cv.rectangle(img1, (50, 100), (100, 150), (0, 255, 0), -1)

    img2 = np.zeros((400, 400, 3), dtype=np.uint8)
    cv.circle(img2, (200, 220), 30, (255, 255, 255), -1)
    cv.rectangle(img2, (80, 120), (130, 170), (0, 255, 0), -1)

    script_dir = Path(__file__).parent
    cv.imwrite(str(script_dir / 'test_frame1.jpg'), img1)
    cv.imwrite(str(script_dir / 'test_frame2.jpg'), img2)

def lucas_kanade_with_two_images(img1_path, img2_path):
    script_dir = Path(__file__).parent
    img1 = cv.imread(str(script_dir / img1_path), cv.IMREAD_COLOR)
    img2 = cv.imread(str(script_dir / img2_path), cv.IMREAD_COLOR)
    if img1 is None or img2 is None:
        print("Error: Could not load images")
        return

    h, w, _ = img1.shape  # NOTE: (rows, cols, ch) -> (H, W, C)

    # --- Pinned host buffers (optional speed-up) ---
    use_hostmem = hasattr(cv, "cuda_HostMem")
    if use_hostmem:
        hmem1 = cv.cuda_HostMem(h, w, cv.CV_8UC3, 1)
        hmem2 = cv.cuda_HostMem(h, w, cv.CV_8UC3, 1)
        hmat1 = hmem1.createMatHeader(); hmat1[:] = img1
        hmat2 = hmem2.createMatHeader(); hmat2[:] = img2

    # --- GPU mats + stream ---
    stream = cv.cuda_Stream()
    d_img1 = cv.cuda_GpuMat()
    d_img2 = cv.cuda_GpuMat()
    d_gray1 = cv.cuda_GpuMat()
    d_gray2 = cv.cuda_GpuMat()

    if use_hostmem:
        d_img1.upload(hmat1, stream=stream)
        d_img2.upload(hmat2, stream=stream)
    else:
        d_img1.upload(img1, stream=stream)
        d_img2.upload(img2, stream=stream)

    # Correct cv.cuda.cvtColor signature: (src, code, dst=..., stream=...)
    d_gray1 = cv.cuda.cvtColor(d_img1, cv.COLOR_BGR2GRAY, stream=stream)
    d_gray2 = cv.cuda.cvtColor(d_img2, cv.COLOR_BGR2GRAY, stream=stream)

    # --- CUDA Good Features To Track (Shi–Tomasi) ---
    detector = cv.cuda.createGoodFeaturesToTrackDetector(
        cv.CV_8UC1, maxCorners=200, qualityLevel=0.01,
        minDistance=7, blockSize=7, useHarrisDetector=False
    )
    d_p0 = detector.detect(d_gray1, stream=stream)  # Nx1 CV_32FC2
    stream.waitForCompletion()
    if d_p0.empty():
        print("No features detected")
        return

    # --- CUDA Sparse LK ---
    lk = cv.cuda.SparsePyrLKOpticalFlow_create(
        winSize=(15, 15), maxLevel=2, iters=10, useInitialFlow=False
    )
    # Order is prev -> next:
    d_p1, d_status, d_err = lk.calc(d_gray1, d_gray2, d_p0, None, stream=stream)
    stream.waitForCompletion()

    # --- Download + filter on CPU ---
    p0 = d_p0.download().reshape(-1, 2)
    p1 = d_p1.download().reshape(-1, 2)
    st = d_status.download().reshape(-1).astype(bool)

    good_old = p0[st]
    good_new = p1[st]

    # --- Draw vectors on img2 ---
    result = img2.copy()
    for (a, b), (c, d) in zip(good_new.astype(int), good_old.astype(int)):
        cv.arrowedLine(result, (c, d), (a, b), (0, 255, 255), 2, tipLength=0.3)
        cv.circle(result, (a, b), 3, (0, 0, 255), -1)
        cv.circle(result, (c, d), 3, (255, 0, 0), -1)

    combined = np.hstack([img1, result])
    cv.imshow("Lucas-Kanade: Before -> After", combined)
    cv.imwrite(str(script_dir / "lk_result.jpg"), result)
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    # create_test_sequence()
    lucas_kanade_with_two_images("test_frame1.jpg", "test_frame2.jpg")
