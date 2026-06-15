import blobconverter

# The path to sliced YOLO26 ONNX file
onnx_path = "/home/omar/Ausra-Autonomous-Systems/AUSRA-Autonomous-System/Perception/tools/shared_with_container/outputs/best_20260411_210246/best.onnx"

# Sending the sliced ONNX file directly to the Luxonis compiler
print("Sending sliced ONNX to Luxonis cloud compiler...")


blob_path = blobconverter.from_onnx(
    model=onnx_path,
    data_type="FP16",  # 16-bit math the camera loves
    shaves=6,          # 6 cores for the OAK-D Lite
    use_cache=False
)

print(f"SUCCESS! Compiled blob is saved here: {blob_path}")