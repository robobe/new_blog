import cv2
import numpy as np

# Check if CUDA is available
if not cv2.cuda.getCudaEnabledDeviceCount():
    print("CUDA is not enabled or no CUDA devices found.")
    exit()

# Define image dimensions and type
rows, cols = 480, 640
image_type = cv2.CV_8UC1 # 8-bit, single channel (grayscale)

# 1. Allocate page-locked host memory
# You can specify the allocation type (PAGE_LOCKED or SHARED)
PAGE_LOCKED = 1
host_mem = cv2.cuda.HostMem(rows, cols, image_type, PAGE_LOCKED)
host_mem_download = cv2.cuda.HostMem(rows, cols, image_type, PAGE_LOCKED)

# 2. Access the allocated memory as a NumPy array (Mat header)
# This creates a NumPy array that shares the underlying memory with HostMem
host_mat = host_mem.createMatHeader()
host_mat_download = host_mem_download.createMatHeader()

# 3. Fill the host_mat with some data (e.g., a simple pattern)
for r in range(rows):
    for c in range(cols):
        host_mat[r, c] = (r + c) % 255

# 4. Upload the data from HostMem to a GpuMat
gpu_mat = cv2.cuda_GpuMat()
gpu_mat.upload(host_mat)

# 5. Perform a CUDA operation (e.g., Gaussian blur)
gaussian_filter = cv2.cuda.createGaussianFilter(image_type, image_type, (5, 5), 0)
gpu_blurred_mat = gaussian_filter.apply(gpu_mat)

# 6. Download the result back to host memory (can be a regular NumPy array)
# 6. Download from GPU
gpu_blurred_mat.download(host_mat_download)


# 7. Display the original and processed images (optional)
cv2.imshow("Original (from HostMem)", host_mat)
cv2.imshow("Blurred (on CPU after GPU processing)", host_mat_download)
cv2.waitKey(0)
cv2.destroyAllWindows()
