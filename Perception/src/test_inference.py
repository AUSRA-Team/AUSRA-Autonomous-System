from ultralytics import YOLO

# 1. Load the trained model
model = YOLO("../weights/best.pt")

# 2. Run inference on the entire test folder

source = "../data/test/images"
results = model.predict(source=source,
                        conf = 0.5,
                        save = True,
                        project = "data",
                        name = "test",
                        exist_ok = True
                        )
print("Inference complete!")