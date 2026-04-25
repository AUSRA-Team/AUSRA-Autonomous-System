import depthai as dai
import cv2
import numpy as np
import time

# --- 1. SYSTEM CONFIGURATION ---
BLOB_PATH = "../weights/best.blob"
IMG_SIZE = 640
CONFIDENCE_THRESHOLD = 0.20  # 40% certainty required to draw a box
NMS_THRESHOLD = 0.40         # 40% overlap threshold for deleting duplicates

print("Booting Ausra Vision System (Optimized Production Release)...")

# --- 2. BUILD THE HARDWARE PIPELINE ---
pipeline = dai.Pipeline()

# Camera Node
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(IMG_SIZE, IMG_SIZE)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setInterleaved(False)
# Ensure OpenCV gets the BGR format it expects
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Neural Network Node (Bypassing XLink limits)
nn = pipeline.create(dai.node.NeuralNetwork)
nn.setBlobPath(BLOB_PATH)
nn.setNumInferenceThreads(2)
nn.input.setBlocking(False)

cam.preview.link(nn.input)

# Video Output Stream
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
nn.passthrough.link(xout_rgb.input)

# Raw AI Data Output Stream
xout_nn = pipeline.create(dai.node.XLinkOut)
xout_nn.setStreamName("nn")
nn.out.link(xout_nn.input)

# --- 3. RUN THE LIVE STREAM ---
with dai.Device(pipeline) as device:
    print("Camera Connected! Ausra System is LIVE.")
    
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

    startTime = time.monotonic()
    counter = 0
    fps = 0

    while True:
        in_rgb = q_rgb.get()
        in_nn = q_nn.get()

        frame = in_rgb.getCvFrame()

        # Calculate Real-Time FPS
        counter += 1
        current_time = time.monotonic()
        if (current_time - startTime) > 1:
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        # HIGH-SPEED NUMPY DECODING
        raw_data = np.array(in_nn.getLayerFp16('output_yolo26'))
        
        # Read the matrix exactly as the AI structured it
        predictions = raw_data.reshape(8400, 7)

        # Extract data columns exactly as they are (4 coords, 1 obj, 2 classes)
        boxes_raw = predictions[:, :4]
        obj_probs = predictions[:, 4:5]     # Native percentage (0.0 to 1.0)
        class_probs = predictions[:, 5:]    # Native percentage (0.0 to 1.0)

        # Calculate final confidence (Object % * Highest Class %)
        final_probs = obj_probs * np.max(class_probs, axis=1, keepdims=True)
        confidences = final_probs.flatten()
        class_ids = np.argmax(class_probs, axis=1)

        # Filter out low-confidence background noise
        mask = confidences > CONFIDENCE_THRESHOLD
        
        filtered_boxes = boxes_raw[mask]
        filtered_confs = confidences[mask]
        filtered_class_ids = class_ids[mask]

        boxes_to_nms = []
        
        if len(filtered_boxes) > 0:
            # PyTorch actually exported absolute corners: [x_min, y_min, x_max, y_max]
            x_mins_raw = filtered_boxes[:, 0]
            y_mins_raw = filtered_boxes[:, 1]
            x_maxs_raw = filtered_boxes[:, 2]
            y_maxs_raw = filtered_boxes[:, 3]
            
            # Calculate standard Width and Height for OpenCV
            widths_raw = x_maxs_raw - x_mins_raw
            heights_raw = y_maxs_raw - y_mins_raw
            
            # OpenCV Safety Net: Neutralize NaN/Inf
            x_mins = np.nan_to_num(x_mins_raw, nan=0, posinf=IMG_SIZE, neginf=0).astype(int)
            y_mins = np.nan_to_num(y_mins_raw, nan=0, posinf=IMG_SIZE, neginf=0).astype(int)
            widths = np.nan_to_num(widths_raw, nan=0, posinf=IMG_SIZE, neginf=0).astype(int)
            heights = np.nan_to_num(heights_raw, nan=0, posinf=IMG_SIZE, neginf=0).astype(int)
            
            # Format for NMS
            for i in range(len(x_mins)):
                boxes_to_nms.append([int(x_mins[i]), int(y_mins[i]), int(widths[i]), int(heights[i])])

            #NON-MAXIMUM SUPPRESSION
            indices = cv2.dnn.NMSBoxes(boxes_to_nms, filtered_confs.tolist(), score_threshold=CONFIDENCE_THRESHOLD, nms_threshold=NMS_THRESHOLD)
            
            if len(indices) > 0:
                for i in indices.flatten():
                    x, y, w, h = boxes_to_nms[i]
                    conf = filtered_confs[i]
                    class_id = filtered_class_ids[i]
                    
                    if class_id == 1:
                        target_name = "Victim"
                        color = (0, 0, 255) # Green
                    else:
                        target_name = "Non-Victim"
                        color = (0, 255, 0) # Red
                        
                    label_text = f"{target_name}: {int(conf * 100)}%"
                    
                    # Draw Final Clean Boxes
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(frame, label_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Display UI
        cv2.putText(frame, f"Ausra System FPS: {fps:.1f}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("Ausra Robot Vision", frame)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()