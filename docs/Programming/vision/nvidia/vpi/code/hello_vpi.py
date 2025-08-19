import sys
import vpi
 
import numpy as np
from PIL import Image
 

try:
    input = vpi.asimage(np.asarray(Image.open("/home/user/projects/nvidia_tutorial/assets/kodim08.png")))
except IOError:
    sys.exit("Input file not found")
except:
    sys.exit("Error with input file")
 
# 4. Convert it to grayscale .
# -----------------------------------------------------------------------------
 
# Enabling the CUDA backend in a python context like done below makes
# VPI algorithms use CUDA for execution by default. This can be overriden
# if needed by specifying the parameter `backend=` when calling the algorithm.
with vpi.Backend.CUDA:
    # `image.convert` will return a new VPI image with the desired format, in
    # this case U8 (grayscale, 8-bit unsigned pixels).
    # Algorithms returning a new VPI image allows for chaining operations like
    # done below, as the result of the conversion is then filtered.
    # The end result is finally stored in a new `output` VPI image.
    output = input.convert(vpi.Format.U8)
 
# -----------------------------------------------------------------------------
 
# The read-lock context enabled below makes sure all processing is finished and
# results are stored in `output`.
with output.rlock_cpu() as outData:
    # `outData` is a numpy array view (not a copy) of `output`
    # contents, accessible by host (cpu). 
    # The numpy array is then converted into a Pillo image and saved / show
    # to the disk
    Image.fromarray(outData).show()
