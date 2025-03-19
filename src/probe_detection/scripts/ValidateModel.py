#!/usr/bin/env python3
import onnx

model_path = "/home/daroe/ERC-Mars-Rover/src/probe_detection/models/many_probes_weights.onnx"
onnx_model = onnx.load(model_path)
onnx.checker.check_model(onnx_model)
print("ONNX model is valid!")
print(f"Opset version: {onnx_model.opset_import[0].version}")