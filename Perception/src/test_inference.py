from ultralytics import YOLO

# 1. Load the trained model
model = YOLO("../runs/YOLO26n_model/weights/best.pt")

# 2. Run inference on the entire test folder

source = "../data/test/images"
results = model.predict(source=source,
                        conf = 0.4,
                        save = True,
                        project = "data2",
                        name = "test2",
                        exist_ok = True,
                        stream=False
                        )
print("Inference complete!")