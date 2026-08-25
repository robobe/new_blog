from rknn.api import RKNN

ONNX_MODEL = "yolo26n.onnx"
DATASET = "calibration.txt"

rknn = RKNN(verbose=True)

ret = rknn.config(
    target_platform="rk3566",
    mean_values=[[0, 0, 0]],
    std_values=[[255, 255, 255]],
)
assert ret == 0

ret = rknn.load_onnx(
    model=ONNX_MODEL
)
assert ret == 0

ret = rknn.hybrid_quantization_step1(
    dataset=DATASET,
    proposal=False,
)
assert ret == 0

rknn.release()