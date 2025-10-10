import onnxruntime as ort

# Show all available providers on your machine
print("Available providers:", ort.get_available_providers())
print("ONNX Runtime version:", ort.__version__)
print("Build info:", ort.get_device())
