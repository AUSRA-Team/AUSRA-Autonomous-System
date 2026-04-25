from ultralytics import YOLO

# Load your model
model = YOLO("../runs/YOLO26n_model/weights/best.pt")

# Run validation specifically on the TEST split
metrics = model.val(
    data="../data/custom_data.yaml",  
    split="test",                       # Bt2ol YOLO to use the test set, not the val set
    project="results/new_model_val",
    name="test_metrics_nano_model"
)

# Print the final score to the terminal
print(f"Test Precision: {metrics.box.mp:.3f}")
print(f"Test Recall:    {metrics.box.mr:.3f}")
print(f"Test mAP50:     {metrics.box.map50:.3f}")
print(f"Test mAP50-95:  {metrics.box.map:.3f}")