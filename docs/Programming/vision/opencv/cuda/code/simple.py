import cv2
import numpy as np

def test_cuda_functionality():
    try:
        # Create a test image
        img = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        
        # Try to upload to GPU
        gpu_img = cv2.cuda_GpuMat()
        gpu_img.upload(img)
        
        # Try a simple operation
        gpu_gray = cv2.cuda.cvtColor(gpu_img, cv2.COLOR_BGR2GRAY)
        
        # Download result
        result = gpu_gray.download()
        
        print("CUDA functionality test: PASSED")
        return True
        
    except Exception as e:
        print(f"CUDA functionality test: FAILED - {e}")
        return False

test_cuda_functionality()