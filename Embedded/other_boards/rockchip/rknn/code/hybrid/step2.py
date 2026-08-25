from rknn.api import RKNN

MODEL = "yolo26n.model"
DATA = "yolo26n.data"
CFG = "yolo26n.quantization.cfg"
OUTPUT = "yolo26n_hybrid.rknn"

rknn = RKNN(verbose=True)

ret = rknn.hybrid_quantization_step2(
    model_input=MODEL,
    data_input=DATA,
    model_quantization_cfg=CFG,
)
assert ret == 0

ret = rknn.export_rknn(OUTPUT)
assert ret == 0

print(f"created: {OUTPUT}")

rknn.release()