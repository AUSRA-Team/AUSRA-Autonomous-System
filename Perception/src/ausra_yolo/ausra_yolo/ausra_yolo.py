import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import numpy as np
from ultralytics import YOLO
import os

class AusraYoloNode(Node):
    def __init__(self):
        super().__init__('ausra_yolo_node')

        # 1. Declare a parameter with a default value of 'ausra_1'
        self.declare_parameter('robot_name', 'ausra_1')
        
        # 2. Read the parameter value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.get_logger().info(f"Initializing YOLO for robot: {self.robot_name}")

        # 3. Dynamically construct the topic names based on the parameter
        camera_topic = f'/{self.robot_name}/{self.robot_name}_rgb_camera/image_raw'
        detection_topic = f'/{self.robot_name}/yolo/detections'
        debug_topic = f'/{self.robot_name}/yolo/debug_image'

        # 4. Initialize YOLO
        model_path = os.path.expanduser('~/ausra_ws/src/AUSRA-Autonomous-System/Perception/runs/YOLO26n_model/weights/best.pt')
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded successfully")
                               
        # 5. Create your image translator (CV Bridge)
        self.bridge = CvBridge()

        # 6. Create the publisher and subscriber
        self.image_subscriber = self.create_subscription(Image, camera_topic, self.image_callback, 10)

        self.detection_pub = self.create_publisher(Detection2DArray, detection_topic, 10)

        self.debug_image_pub = self.create_publisher(Image, debug_topic, 10)

        self.get_logger().info(f"Listening to: {camera_topic}")

    # 7. Creating the callback function

    def image_callback(self, msg):
        try:
           
            
            # Use the translator to convert incoming_ros_image into an OpenCV array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Pass the OpenCV array into the YOLO model to get the results
            results = self.model(cv_image, conf=0.4, imgsz = 640, 
                                verbose=False, device = 0, half=True)
        
            # Save the exact timestamp from incoming_ros_image
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # Process each detection
            for box in results[0].boxes:
        
                detection = Detection2D()
                
                # Get bounding box coordinates
                center_x, center_y, width, height = box.xywh[0].cpu().numpy()

                # Set bounding box center and size
                detection.bbox.center.position.x = float(center_x)
                detection.bbox.center.position.y = float(center_y)
                detection.bbox.size_x = float(width)
                detection.bbox.size_y = float(height)

                # Extract the Class ID and score
                label = self.model.names[int(box.cls[0])]  
                score = float(box.conf[0])

                # Set detection hypothesis (class and confidence)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(label)
                hypothesis.hypothesis.score = score
                    
                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

            # Plotting the bounding box
            annotated_frame = results[0].plot()

            # Publish detections and debug images
            self.detection_pub.publish(detection_array)

            debug_img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            debug_img_msg.header = msg.header
            self.debug_image_pub.publish(debug_img_msg)

            self.get_logger().debug(f"Published {len(detection_array.detections)} detections")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
def main(args=None):
    rclpy.init(args=args)
    node = AusraYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()