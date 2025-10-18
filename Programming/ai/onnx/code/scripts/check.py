import onnxruntime as ort

ort.set_default_logger_severity(0)  # 0 = VERBOSE, 1 = INFO, 2 = WARNING, 3 = ERROR, 4 = FATAL


# Show all available providers on your machine
print("Available providers:", ort.get_available_providers())
print("ONNX Runtime version:", ort.__version__)
print("Build info:", ort.get_device())
