import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ConeDetectorNode(Node):
    """
    Performs object detection on camera images using YOLOv8

    Output:
    - Publishes `Detection2DArray` containing bounding boxes and class IDs
    - Publishes a debug image with drawn boxes for visualization
    """
    def __init__(self):
        super().__init__('camera_detector')

        # Declare parameters
        self.declare_parameter('model_path', '/home/sir/ros2_ws/yolo/cone_detection/yolov8n_run2/weights/best.pt')
        self.declare_parameter('topics.sub_image', '/carla/hero/rgb_front/image')
        self.declare_parameter('topics.pub_bounding_boxes', '/perception/camera/bounding_boxes')
        self.declare_parameter('topics.pub_debug_boxes', '/perception/camera/debug_image')
        self.declare_parameter('confidence_threshold', 0.4)

        # Read Parameters
        # Model path
        model_path = self.get_parameter('model_path').value

        # Topic Names
        sub_image_topic = self.get_parameter('topics.sub_image').value
        pub_boxes_topic = self.get_parameter('topics.pub_bounding_boxes').value
        pub_debug_topic = self.get_parameter('topics.pub_debug_boxes').value

        # Minimum Confidence score (0.0 - 1.0) to accept detection
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        # Load YOLO Model
        self.get_logger().info(f'Loading YOLO from {model_path}...')
        self.model = YOLO(model_path)
        self.model_names = self.model.names

        # Setup ros msg converter
        self.bridge = CvBridge()

        # Setup Subscriber
        self.subscription = self.create_subscription(
            Image,
            sub_image_topic,
            self.image_callback,
            10
        )

        # Setup Publishers
        self.box_publisher = self.create_publisher(
            Detection2DArray,
            pub_boxes_topic,
            10
        )

        self.debug_publisher = self.create_publisher(
            Image,
            pub_debug_topic,
            10
        )

        self.get_logger().info('Camera Node Initialized')

    def image_callback(self, msg):
        # Convert Ros Image to OpenCV Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Prediction
        results = self.model(cv_image, iou=0.45, rect=True, conf=self.conf_threshold, verbose=False)

        # Message for fusion
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        # Message for debugging
        debug_img = cv_image.copy()

        # Parse Results
        # results[0].boxes contains the list of all detected objects
        for box in results[0].boxes:
            # Coordinates: Top-Left (x1, y1) and Bottom-Right (x2, y2)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            # Calculate Center and size
            width = float(x2 - x1)
            height = float(y2 - y1)
            cx = x1 + (width / 2.0)
            cy = y1 + (height / 2.0)

            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = float(cx)
            detection.bbox.center.position.y = float(cy)
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)

            # Define Class
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(cls_id)
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

            # Draw Debug Visualization
            color = (0, 255, 0)
            if cls_id == 0: color = (0, 255, 255)  # Yellow
            elif cls_id == 1: color = (255, 0, 0)  # Blue
            elif cls_id == 4: color = (0, 0, 0)    # Black
            else: color = (0, 165, 255)            # Orange

            cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            label = f"{self.model_names[cls_id]} {conf:.2f}"
            cv2.putText(debug_img, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publish
        self.box_publisher.publish(detections_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_publisher.publish(debug_msg)

        # self.get_logger().info("Image published...")


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()