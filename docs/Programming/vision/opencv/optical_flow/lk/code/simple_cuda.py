import cv2
import numpy as np
from pathlib import Path

def create_test_sequence():
    """
    Create simple test images to demonstrate optical flow
    """
    # Create first image with a white circle
    img1 = np.zeros((400, 400, 3), dtype=np.uint8)
    cv2.circle(img1, (150, 200), 30, (255, 255, 255), -1)
    cv2.rectangle(img1, (50, 100), (100, 150), (0, 255, 0), -1)
    
    # Create second image with moved objects
    img2 = np.zeros((400, 400, 3), dtype=np.uint8)  
    cv2.circle(img2, (200, 220), 30, (255, 255, 255), -1)  # Circle moved right+down
    cv2.rectangle(img2, (80, 120), (130, 170), (0, 255, 0), -1)  # Rectangle moved right+down
    
    # Save test images
    script_dir = Path(__file__).parent
    cv2.imwrite(script_dir / 'test_frame1.jpg', img1)
    cv2.imwrite(script_dir / 'test_frame2.jpg', img2)

def lucas_kanade_with_two_images(img1_path, img2_path):
    """
    Lucas-Kanade with two specific images
    """
    # Load two images
    script_dir = Path(__file__).parent
    img1 = cv2.imread(script_dir / img1_path)
    img2 = cv2.imread(script_dir / img2_path)

    if img1 is None or img2 is None:
        print("Error: Could not load images")
        return
    
    w, h, _ = img1.shape
    # Allocate pinned host memory for faster CPU-GPU transfers
    PAGE_LOCKED = 1
    pinned_img1 = cv2.cuda.HostMem(h, w, cv2.CV_8UC3, PAGE_LOCKED)
    hmat1 = pinned_img1.createMatHeader() 
    pinned_img2 = cv2.cuda.HostMem(h, w, cv2.CV_8UC3, PAGE_LOCKED)
    hmat2 = pinned_img2.createMatHeader() 

    gpu_img1 = cv2.cuda.GpuMat()
    gpu_img2 = cv2.cuda.GpuMat()
    gpu_gray1 = cv2.cuda.GpuMat()
    gpu_gray2 = cv2.cuda.GpuMat()
    
    hmat1[:] = img1
    hmat2[:] = img2

    # TODO:         self.gpu_img1.upload_async(self.pinned_img1.ptr(), self.stream)
    gpu_img1.upload(hmat1)
    gpu_img2.upload(hmat2)

    # Convert to grayscale on GPU
    gpu_gray1 = cv2.cuda.cvtColor(gpu_img1, cv2.COLOR_BGR2GRAY)
    gpu_gray2 = cv2.cuda.cvtColor(gpu_img2, cv2.COLOR_BGR2GRAY)
        

    gpu_detector = cv2.cuda.createGoodFeaturesToTrackDetector(
            cv2.CV_8UC1,
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )
    
    # Detect features in first image
    p0_gpu = gpu_detector.detect(gpu_gray1)
    
    
    
    # Calculate optical flow
    lk = cv2.cuda.SparsePyrLKOpticalFlow_create(
        winSize=(15, 15), 
        maxLevel=2,
        iters=10, 
        useInitialFlow=False
    )
    
    p1_gpu, status_gpu, error_gpu = lk.calc(
        gpu_gray1,
        gpu_gray2, 
        p0_gpu,
        None,)
    p0 = p0_gpu.download().reshape(-1, 2)
    p1 = p1_gpu.download().reshape(-1, 2)
    status = status_gpu.download().reshape(-1).astype(bool)

    if p0 is None:
        print("No features detected")
        return
    
    # Filter good points
    good_new = p1[status == 1]
    good_old = p0[status == 1]
    
    
    # Draw motion vectors
    result = img2.copy()
    for new, old in zip(good_new, good_old):
        a, b = new.ravel().astype(int)
        c, d = old.ravel().astype(int)
        
        # Draw arrow showing motion
        cv2.arrowedLine(result, (c, d), (a, b), (0, 255, 255), 2, tipLength=0.3)
        cv2.circle(result, (a, b), 3, (0, 0, 255), -1)  # Current position
        cv2.circle(result, (c, d), 3, (255, 0, 0), -1)  # Previous position
    
    # Display both images
    combined = np.hstack([img1, result])
    cv2.imshow('Lucas-Kanade: Before -> After', combined)
    script_dir = Path(__file__).parent
    cv2.imwrite(script_dir / 'lk_result.jpg', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # create_test_sequence()
    lucas_kanade_with_two_images("test_frame1.jpg",
                                 "test_frame2.jpg")