import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
from sensor_msgs.msg import CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import sensor_msgs_py.point_cloud2 as pc2
from interfaces.msg import Cone, ConeArray
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

class SensorFusionNode(Node):
    """
    Fuses 3D Lidar LiDAR centroids with 2D Camera Bounding Boxes

    Approach:
    1. Synchronize Lidar and Camera messages
    2. Transform Lidar points into the Camera coordinate frame
    3. Project 3D Lidar points onto the 2D image plane using camera intrinsics
    4. Check if the projected pixel lies inside a 2D Bounding Box
    5. If yes, assign the class (Color) of the box to the 3D position
    """
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Declare parameters
        self.declare_parameter('topics.camera_boxes', '/perception/camera/bounding_boxes')
        self.declare_parameter('topics.lidar_centroids', '/perception/lidar')
        self.declare_parameter('topics.perception_cones', '/perception/cone_list')
        self.declare_parameter('topics.camera_info', '/carla/hero/rgb_front/camera_info')
        self.declare_parameter('topics.perception_markers', '/perception/markers')
        self.declare_parameter('sync_slop', 0.15)

        # Read parameters
        # Topic names
        sub_boxes_topic = self.get_parameter('topics.camera_boxes').value
        sub_lidar_topic = self.get_parameter('topics.lidar_centroids').value
        sub_camera_info = self.get_parameter('topics.camera_info').value
        pub_cone_list = self.get_parameter('topics.perception_cones').value
        pub_markers = self.get_parameter('topics.perception_markers').value

        # Synchronizer slop
        sync_slop = self.get_parameter('sync_slop').value

        # TF Buffer for coordinate transforms (Lidar Frame -> Camera Frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera Intrinsics holder
        self.camera_info = None
        self.create_subscription(CameraInfo, sub_camera_info, self.camera_info_cb, 10)

        # Time Synchronizer
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, sub_lidar_topic, 10)
        self.camera_sub = message_filters.Subscriber(self, Detection2DArray, sub_boxes_topic, 10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.camera_sub],
            queue_size=100,
            slop=sync_slop
        )
        self.ts.registerCallback(self.fusion_callback)

        # Publishers
        self.cone_pub = self.create_publisher(ConeArray, pub_cone_list, 10)
        self.marker_pub = self.create_publisher(MarkerArray, pub_markers, 10)

        self.get_logger().info("Sensor Fusion Node Started")

    def camera_info_cb(self, msg):
        """ Stores camera intrinsic matrix parameters (fx, fy, cx, cy) """
        if self.camera_info is not None: return

        self.camera_info = msg
        self.fx = msg.k[0] # Focal length x
        self.cx = msg.k[2] # Principal point x
        self.fy = msg.k[4] # Focal length y
        self.cy = msg.k[5] # Principal point y

    def fusion_callback(self, lidar_msg, bbox_msg):
        """ Main Fusion Loop """
        if self.camera_info is None:
            self.get_logger().info('Waiting for camera info')
            return

        # Transform Lidar Cloud to Camera Frame
        try:
            transform = self.tf_buffer.lookup_transform(
                bbox_msg.header.frame_id,
                lidar_msg.header.frame_id,
                rclpy.time.Time()
            )
            cloud_camera_frame = do_transform_cloud(lidar_msg, transform)
        except Exception as e:
            self.get_logger().info(f"TF Error: {e}")
            return

        fused_cones = []

        # Generators for iterating over points
        # gen_camera: Points in camera frame
        # gen_lidar: Points in lidar frame
        gen_camera = pc2.read_points(cloud_camera_frame, field_names=('x', 'y', 'z'), skip_nans=True)
        gen_lidar = pc2.read_points(lidar_msg, field_names=('x', 'y', 'z'), skip_nans=True)

        for cam_pt, lidar_pt in zip(gen_camera, gen_lidar):
            cam_x, cam_y, cam_z = cam_pt

            # Skip points behind the camera
            if cam_z <= 0: continue

            # Project 3D point to 2D pixel
            # Equation: u = (fx + x) / z + cx
            #           v = (fy * y) / z + cy
            u = int((self.fx * cam_x) / cam_z + self.cx)
            v = int((self.fy * cam_y) / cam_z + self.cy)

            # Match with Bounding Box
            match_result = self.get_info_from_bbox(u, v, bbox_msg.detections)

            if match_result is not None:
                color_id, confidence = match_result
                orig_x, orig_y, orig_z = lidar_pt
                fused_cones.append((orig_x, orig_y, orig_z, color_id, confidence))


        # Publish results
        self.publish_cones(fused_cones, lidar_msg.header)
        self.publish_markers(fused_cones, lidar_msg.header.frame_id)

        # self.get_logger().info("ConeArray published")

    def get_info_from_bbox(self, u, v, detections):
        """ Checks if pixel (u, v) falls within any of the detected bounding boxes """
        for detection in detections:
            bbox = detection.bbox

            cx, cy = bbox.center.position.x, bbox.center.position.y
            sx, sy = bbox.size_x, bbox.size_y

            # Calculate Box Corners
            x_min = cx - (sx / 2.0)
            x_max = cx + (sx / 2.0)
            y_min = cy - (sy / 2.0)
            y_max = cy + (sy / 2.0)

            # Check if is inside
            if x_min <= u <= x_max and y_min <= v <= y_max:
                if detection.results:
                    hyp = detection.results[0].hypothesis
                    return (hyp.class_id, hyp.score)

        return None

    def publish_markers(self, cones, frame_id):
        """ Publishes Rviz markers for visualization """
        marker_array = MarkerArray()

        # Delete previous markers in Rviz
        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, (x, y, z, color_id, _) in enumerate(cones):
            m = Marker()
            m.header.frame_id = frame_id
            m.id = i + 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.4
            m.color.a = 0.4

            m.pose.orientation.w = 1.0;

            # Map class_id to color
            if int(color_id) == 0:   # Yellow
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            elif int(color_id) == 1: # Blue
                m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0
            elif int(color_id) == 4: # Unknown
                m.color.r, m.color.g, m.color.b = 0.0, 0.0, 0.0
            else:                    # Orange
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0

            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    def publish_cones(self, cones, header):
        msg = ConeArray()
        msg.header = header

        for (x, y, z, color_id, confidence) in cones:
            c = Cone()
            c.position.x = float(x)
            c.position.y = float(y)
            c.position.z = float(z)
            c.color = int(color_id)
            c.confidence = float(confidence)

            msg.cones.append(c)

        self.cone_pub.publish(msg)

def main():
    rclpy.init()
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()