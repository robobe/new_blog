# Rockchip RGA Basics: A Hands-On Tutorial for Image Processing and RKNN

Rockchip's **RGA (Raster Graphic Acceleration)** engine is a 2D hardware accelerator designed for common image-processing operations such as:

- Resize
- Crop
- Color conversion
- Copy
- Fill
- Rotation
- Combined image-processing operations

For Edge AI applications, RGA is especially useful because image preprocessing can be moved away from the CPU.

A typical pipeline can look like this:

```text
Camera / OpenCV
      |
      v
     RGA
 crop / resize
 BGR -> RGB
      |
      v
 RKNN input buffer
      |
      v
     NPU
```

This tutorial focuses on the basic RGA concepts first, then gradually moves toward integration with RKNN.

---

## 1. The RGA Mental Model

The easiest way to understand RGA is as a four-step process:

```text
Allocate memory
      |
      v
Import memory into RGA
      |
      v
Describe the image buffer
      |
      v
Run an RGA operation
```

For example, suppose we have a normal C++ image buffer:

```cpp
uint8_t* data = ...;
```

The modern `im2d` flow is approximately:

```cpp
rga_buffer_handle_t handle =
    importbuffer_virtualaddr(data, size);

rga_buffer_t image =
    wrapbuffer_handle(
        handle,
        width,
        height,
        RK_FORMAT_BGR_888);

imresize(...);

releasebuffer_handle(handle);
```

Conceptually:

```text
uint8_t*
cv::Mat.data
DMA-BUF fd
      |
      | importbuffer_*
      v
rga_buffer_handle_t
      |
      | wrapbuffer_handle()
      v
rga_buffer_t
      |
      v
imresize / imcopy / imcvtcolor / improcess
```

The two important objects are:

- `rga_buffer_handle_t` — represents memory imported into RGA.
- `rga_buffer_t` — describes the image stored in that memory.

---

# Hands-On 1: Minimal RGA Resize

Start without OpenCV and without RKNN.

The goal is simply:

```text
640x480 BGR
    |
    | RGA
    v
320x240 BGR
```

```cpp
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <rga/im2d.hpp>
#include <rga/RgaUtils.h>

int main()
{
    constexpr int src_w = 640;
    constexpr int src_h = 480;

    constexpr int dst_w = 320;
    constexpr int dst_h = 240;

    constexpr int channels = 3;

    const int src_size =
        src_w * src_h * channels;

    const int dst_size =
        dst_w * dst_h * channels;

    auto* src =
        static_cast<uint8_t*>(
            std::malloc(src_size));

    auto* dst =
        static_cast<uint8_t*>(
            std::malloc(dst_size));

    if (!src || !dst) {
        std::cerr << "malloc failed\n";
        return 1;
    }

    std::memset(src, 128, src_size);
    std::memset(dst, 0, dst_size);

    // 1. Import CPU memory into RGA
    rga_buffer_handle_t src_handle =
        importbuffer_virtualaddr(
            src,
            src_size);

    rga_buffer_handle_t dst_handle =
        importbuffer_virtualaddr(
            dst,
            dst_size);

    if (!src_handle || !dst_handle) {
        std::cerr << "RGA import failed\n";
        return 1;
    }

    // 2. Describe the image buffers
    rga_buffer_t src_img =
        wrapbuffer_handle(
            src_handle,
            src_w,
            src_h,
            RK_FORMAT_BGR_888);

    rga_buffer_t dst_img =
        wrapbuffer_handle(
            dst_handle,
            dst_w,
            dst_h,
            RK_FORMAT_BGR_888);

    // 3. Run the resize
    IM_STATUS status =
        imresize(
            src_img,
            dst_img);

    if (status != IM_STATUS_SUCCESS) {
        std::cerr
            << "imresize failed: "
            << imStrError(status)
            << '\n';
    } else {
        std::cout
            << "RGA resize succeeded\n";
    }

    // 4. Release RGA resources
    releasebuffer_handle(src_handle);
    releasebuffer_handle(dst_handle);

    std::free(src);
    std::free(dst);

    return 0;
}
```

The important APIs are:

```cpp
importbuffer_virtualaddr()
wrapbuffer_handle()
imresize()
releasebuffer_handle()
```

Everything else is ordinary C++ memory management.

---

# Hands-On 2: OpenCV + RGA

A very useful learning setup is:

- OpenCV loads the image.
- RGA performs the resize.
- OpenCV saves the result.

This isolates RGA from RKNN and makes debugging much easier.

```cpp
#include <iostream>

#include <opencv2/opencv.hpp>

#include <rga/im2d.hpp>
#include <rga/RgaUtils.h>

int main()
{
    cv::Mat src =
        cv::imread("input.jpg");

    if (src.empty()) {
        std::cerr
            << "Cannot open input.jpg\n";
        return 1;
    }

    if (src.type() != CV_8UC3) {
        std::cerr
            << "Expected BGR CV_8UC3 image\n";
        return 1;
    }

    constexpr int dst_w = 320;
    constexpr int dst_h = 320;

    cv::Mat dst(
        dst_h,
        dst_w,
        CV_8UC3);

    const int src_size =
        static_cast<int>(
            src.step * src.rows);

    const int dst_size =
        static_cast<int>(
            dst.step * dst.rows);

    auto src_handle =
        importbuffer_virtualaddr(
            src.data,
            src_size);

    auto dst_handle =
        importbuffer_virtualaddr(
            dst.data,
            dst_size);

    if (!src_handle ||
        !dst_handle) {

        std::cerr
            << "RGA import failed\n";

        return 1;
    }

    auto src_img =
        wrapbuffer_handle(
            src_handle,
            src.cols,
            src.rows,
            RK_FORMAT_BGR_888);

    auto dst_img =
        wrapbuffer_handle(
            dst_handle,
            dst.cols,
            dst.rows,
            RK_FORMAT_BGR_888);

    IM_STATUS status =
        imresize(
            src_img,
            dst_img);

    releasebuffer_handle(src_handle);
    releasebuffer_handle(dst_handle);

    if (status != IM_STATUS_SUCCESS) {
        std::cerr
            << "RGA resize failed: "
            << imStrError(status)
            << '\n';

        return 1;
    }

    cv::imwrite(
        "resized.jpg",
        dst);

    return 0;
}
```

The data path is now:

```text
cv::imread
    |
    v
cv::Mat.data
    |
    v
importbuffer_virtualaddr
    |
    v
RGA imresize
    |
    v
cv::Mat.data
    |
    v
cv::imwrite
```

This is a good first real test on an RK3566 board.

---

# Hands-On 3: Understanding Stride

Stride is one of the most important RGA concepts.

Suppose a neural network expects:

```text
255 x 255 BGR
```

Each row logically contains:

```text
255 x 3 = 765 bytes
```

However, hardware often requires row alignment.

On an RK3566 with RGA2, a useful aligned width for a 255-pixel BGR image is:

```text
255 -> 256
```

For example:

```cpp
int logical_width = 255;

int stride_width =
    (logical_width + 3) & ~3;
```

The result is:

```text
logical width = 255
stride width  = 256
```

The memory layout becomes:

```text
Logical image:

<--------- 255 pixels --------->


Actual memory row:

<---------- 256 pixels ---------->

[ useful pixels ][padding]
0              254 255
```

Allocate enough memory for the stride:

```cpp
constexpr int width = 255;
constexpr int height = 255;

constexpr int stride_width =
    (width + 3) & ~3;

const size_t bytes =
    stride_width *
    height *
    3;

std::vector<uint8_t> data(bytes);
```

Then tell RGA both the logical image size and the memory stride:

```cpp
auto handle =
    importbuffer_virtualaddr(
        data.data(),
        bytes);

auto image =
    wrapbuffer_handle(
        handle,
        width,
        height,
        RK_FORMAT_BGR_888,
        stride_width,
        height);
```

The important distinction is:

```text
width
```

means the useful image width.

```text
width stride
```

means the real distance between the beginning of one row and the beginning of the next row.

For NanoTrack-style input:

```text
logical width = 255
stride width  = 256
```

is a normal arrangement.

---

# Hands-On 4: Resize to 255x255 With an Aligned Destination

This example is directly useful for NanoTrack and other models with odd input dimensions.

```cpp
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rga/im2d.hpp>
#include <rga/RgaUtils.h>

int main()
{
    cv::Mat src =
        cv::imread("input.jpg");

    if (src.empty()) {
        std::cerr << "Cannot load image\n";
        return 1;
    }

    constexpr int dst_w = 255;
    constexpr int dst_h = 255;

    constexpr int dst_stride_w =
        (dst_w + 3) & ~3;

    const size_t dst_bytes =
        static_cast<size_t>(
            dst_stride_w) *
        dst_h *
        3;

    std::vector<uint8_t>
        dst_memory(dst_bytes);

    const size_t src_bytes =
        src.step *
        src.rows;

    auto src_handle =
        importbuffer_virtualaddr(
            src.data,
            src_bytes);

    auto dst_handle =
        importbuffer_virtualaddr(
            dst_memory.data(),
            dst_bytes);

    if (!src_handle ||
        !dst_handle) {

        std::cerr
            << "RGA import failed\n";

        return 1;
    }

    auto src_img =
        wrapbuffer_handle(
            src_handle,
            src.cols,
            src.rows,
            RK_FORMAT_BGR_888,
            src.step / 3,
            src.rows);

    auto dst_img =
        wrapbuffer_handle(
            dst_handle,
            dst_w,
            dst_h,
            RK_FORMAT_BGR_888,
            dst_stride_w,
            dst_h);

    IM_STATUS ret =
        imresize(
            src_img,
            dst_img);

    releasebuffer_handle(
        src_handle);

    releasebuffer_handle(
        dst_handle);

    if (ret != IM_STATUS_SUCCESS) {

        std::cerr
            << "RGA failed: "
            << imStrError(ret)
            << '\n';

        return 1;
    }

    cv::Mat dst(
        dst_h,
        dst_w,
        CV_8UC3,
        dst_memory.data(),
        dst_stride_w * 3);

    cv::imwrite(
        "output.jpg",
        dst);

    return 0;
}
```

The destination now has:

```text
logical image: 255 x 255
memory stride: 256 pixels
```

This is an important pattern when working with neural-network input sizes.

---

# Hands-On 5: Cropping With RGA

RGA can process only part of the source image.

For example:

```cpp
im_rect src_rect{
    100,
    50,
    300,
    300
};
```

This means:

```text
source image
+--------------------------------+
|                                |
|     x=100,y=50                 |
|       +---------------+        |
|       |               |        |
|       |    300x300    |        |
|       |               |        |
|       +---------------+        |
|                                |
+--------------------------------+
```

The destination region might be:

```cpp
im_rect dst_rect{
    0,
    0,
    255,
    255
};
```

You can then use `improcess()`:

```cpp
IM_STATUS ret =
    improcess(
        src_img,
        dst_img,
        {},
        src_rect,
        dst_rect,
        {},
        -1,
        nullptr,
        nullptr,
        IM_SYNC);
```

Conceptually:

```text
1920 x 1080
+--------------------------------+
|                                |
|       +------------+           |
|       |    crop    |           |
|       |  300x300   |           |
|       +------------+           |
|                                |
+--------------------------------+
              |
              | RGA
              v
       +---------------+
       |               |
       |    255x255    |
       |               |
       +---------------+
```

This is very useful for tracking applications where every frame requires extracting a search region around the current target position.

---

# Hands-On 6: BGR to RGB

OpenCV normally stores color images as:

```text
B G R
```

Many neural networks expect:

```text
R G B
```

Instead of using:

```cpp
cv::cvtColor(
    src,
    dst,
    cv::COLOR_BGR2RGB);
```

RGA can perform the conversion:

```cpp
IM_STATUS ret =
    imcvtcolor(
        src_img,
        dst_img,
        RK_FORMAT_BGR_888,
        RK_FORMAT_RGB_888);
```

This lets more of the neural-network preprocessing stay on the hardware accelerator.

---

# Hands-On 7: DMA-BUF and RKNN

So far we imported CPU memory with:

```cpp
importbuffer_virtualaddr(...)
```

But RKNN can allocate memory that exposes a DMA-BUF file descriptor.

For example:

```cpp
rknn_tensor_mem* mem =
    rknn_create_mem(
        context,
        buffer_size);
```

The returned structure contains useful fields such as:

```cpp
mem->virt_addr
mem->fd
```

The DMA-BUF file descriptor can be imported into RGA:

```cpp
rga_buffer_handle_t handle =
    importbuffer_fd(
        mem->fd,
        buffer_size);
```

Then describe it:

```cpp
auto dst =
    wrapbuffer_handle(
        handle,
        width,
        height,
        RK_FORMAT_RGB_888,
        stride_width,
        height);
```

Now RGA can potentially write directly into RKNN input memory.

The desired architecture becomes:

```text
OpenCV frame
      |
      | virtual address
      v
importbuffer_virtualaddr()
      |
      v
     RGA
 crop / resize
 BGR -> RGB
      |
      | DMA-BUF
      v
importbuffer_fd()
      |
      v
RKNN input memory
      |
      v
     NPU
```

This avoids an extra CPU copy.

Instead of:

```text
RGA
 |
 v
CPU buffer
 |
 | memcpy
 v
RKNN buffer
 |
 v
NPU
```

we want:

```text
RGA
 |
 +-----------------> RKNN input buffer
                           |
                           v
                          NPU
```

This is the basic idea behind a zero-copy preprocessing pipeline.

---

# RGA APIs Worth Learning First

You do not need to learn the entire librga API before using it effectively.

For computer vision and Edge AI, I recommend starting with these:

| API | Purpose |
|---|---|
| `importbuffer_virtualaddr()` | Import normal CPU/OpenCV memory |
| `importbuffer_fd()` | Import DMA-BUF memory |
| `wrapbuffer_handle()` | Describe an image stored in imported memory |
| `releasebuffer_handle()` | Release an imported RGA handle |
| `imcopy()` | Simple image copy |
| `imresize()` | Resize an image |
| `imcvtcolor()` | Convert BGR/RGB/YUV formats |
| `imfill()` | Fill an image or region |
| `improcess()` | Combined crop, resize, conversion, and other operations |

A good learning order is:

```text
1. import / wrap / release
           |
           v
2. imcopy
           |
           v
3. imresize
           |
           v
4. stride and alignment
           |
           v
5. imcvtcolor
           |
           v
6. improcess + crop
           |
           v
7. DMA-BUF
           |
           v
8. RKNN zero-copy preprocessing
```

---

# RK3566 Notes

The RK3566 uses an RGA2-class engine.

On my RK3566 system, the hardware information can be checked with:

```bash
sudo cat /sys/kernel/debug/rkrga/hardware
```

Example:

```text
rga2, core 4: version: 3.2.63318
input range: 2x2 ~ 8192x8192
output range: 2x2 ~ 4096x4096
scale limit: 1/16 ~ 16
byte_stride_align: 4
max_byte_stride: 32768
mmu: RGA_MMU
```

The kernel driver version can be checked with:

```bash
sudo cat /sys/kernel/debug/rkrga/driver_version
```

For example:

```text
RGA multicore Device Driver: v1.3.5
```

These commands are useful when debugging RGA errors.

---

# Debugging RGA

RGA userspace errors can sometimes be vague.

For example:

```text
RGA_BLIT fail: Device or resource busy
```

does not always mean the hardware is literally busy.

When RGA reports:

```text
Failed to call RockChipRga interface,
please use 'dmesg' command to view driver error log.
```

check the kernel log immediately:

```bash
sudo dmesg | grep -i -E "rga|rkrga" | tail -50
```

or monitor it live:

```bash
sudo dmesg -w
```

Also print the information about your RGA buffers:

```text
logical width
logical height
width stride
height stride
format
fd
virtual address
```

A typical NanoTrack destination could look like:

```text
width       = 255
height      = 255
width stride = 256
format      = BGR888
```

The difference between **logical image size** and **memory stride** is one of the first things to verify when debugging RGA.

---

# Recommended Learning Project

Before combining RGA with RKNN, build this small program first:

```text
JPEG image
    |
    v
OpenCV imread()
    |
    v
RGA
640x480 -> 255x255
stride 256
    |
    v
OpenCV imwrite()
```

Verify that this works reliably.

Then change only the destination:

```text
OpenCV / CPU memory
       |
       v
      RGA
       |
       v
RKNN DMA-BUF
```

This separates two possible classes of problems:

1. RGA image-processing problems.
2. DMA-BUF / RKNN integration problems.

Once both parts work independently, combine them into the final preprocessing pipeline:

```text
Camera
   |
   v
RGA crop
   |
   v
RGA resize
   |
   v
RGA BGR -> RGB
   |
   v
RKNN DMA-BUF
   |
   v
NPU inference
```

---

# References

- Radxa RGA Usage Guide  
  https://docs.radxa.com/en/rock5/rock5b/app-development/rga-usage-guide

- Rockchip librga  
  https://github.com/airockchip/librga

- Rockchip RGA Developer Guide  
  https://github.com/airockchip/librga/blob/main/docs/Rockchip_Developer_Guide_RGA_EN.md
