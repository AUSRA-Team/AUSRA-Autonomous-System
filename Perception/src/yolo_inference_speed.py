import os
import torch
from ultralytics import YOLO

# Load the model
model = YOLO("../runs/YOLO26n/weights/best.pt").to("cuda")

image_dir = "../data/test/images"
images = [os.path.join(image_dir, f) for f in os.listdir(image_dir)]

# 1. Warm-up (Crucial for CUDA)
print("Warming up GPU...")
for _ in range(5):
    model.predict(source=images[0], save=False, verbose=False)

# 2. Run Predictions Safely (One by One)
print(f"Benchmarking {len(images)} images safely...")

inference_times = []
preprocess_times = []
postprocess_times = []

for img in images:
    # Process exactly ONE image at a time to save VRAM
    # [0] grabs the specific result object for this single image
    result = model.predict(source=img, save=False, verbose=False)[0]

    # Extract the exact times from Ultralytics' internal micro-timers
    preprocess_times.append(result.speed['preprocess'])
    inference_times.append(result.speed['inference'])
    postprocess_times.append(result.speed['postprocess'])

# 3. Calculate Averages
avg_pre = sum(preprocess_times) / len(preprocess_times)
avg_inf = sum(inference_times) / len(inference_times)
avg_post = sum(postprocess_times) / len(postprocess_times)
total_pipeline = avg_pre + avg_inf + avg_post

print("\n--- PERFORMANCE BREAKDOWN ---")
print(f"Pre-process (CPU/RAM): {avg_pre:.2f} ms")
print(f"Inference (GPU Math):  {avg_inf:.2f} ms  <-- This is your true model speed!")
print(f"Post-process (NMS):    {avg_post:.2f} ms")
print("-----------------------------")
print(f"Total Pipeline:        {total_pipeline:.2f} ms")
print(f"Max Theoretical FPS (Inference Only): {1000 / avg_inf:.2f} FPS")