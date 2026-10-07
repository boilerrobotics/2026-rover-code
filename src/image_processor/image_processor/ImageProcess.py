import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
    Duration,
)
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Polygon, Point32
import cv2
import numpy as np

class ArUcoNode(Node):
    def __init__(self):
        self.depth_image = None
        self.corners = None
        self.ids = None
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.qos_sub = QoSProfile(
            history=HistoryPolicy.KEEP_ALL
        )
        self.subscription_image = self.create_subscription(
            Image,
            'zed/zed_node/left/color/rect/image',
            self.image_callback,
            qos_profile_system_default)
        self.subscription_depth = self.create_subscription(
            Image,
            'zed/zed_node/depth/depth_registered',
            self.depth_callback,
            qos_profile_system_default)
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            'zed/zed_node/left/color/rect/camera_info',
            self.camera_info_callback,
            qos_profile_system_default)
        self.publisher_ = self.create_publisher(Polygon, '/aruco_locations', qos_profile_system_default)

    def image_callback(self, msg):
        if msg is not None:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            self.corners, self.ids, self.rejected = detector.detectMarkers(gray)

    # turns ros data to OpenCV image format and then detects ArUco markers
    

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.coords = np.zeros((len(self.ids), 3))
        if self.ids is None or self.corners is None:
             return   
        for i in range(len(self.ids)):
            top_left = self.corners[i][0]
            # gets depth value at the marker's location
            u = int(round(top_left[0]))
            v = int(round(top_left[1]))
            depth_value = self.depth_image[v, u]
            self.coords[i] = [u, v, depth_value]
        self.Aruco_publisher()
            
    
    def camera_info_callback(self, msg):
        # Process camera info constantly
        if msg is not None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def ArUco_publisher(self):
        polygon = Polygon()
        for i in range(len(self.coords)):
            point = Point32()
            point.x = float(self.coords[i][2])
            point.y = float((self.cx - self.coords[i][0]) / self.fx * self.coords[i][2])
            point.z = float((self.cy - self.coords[i][1]) / self.fy * self.coords[i][2])
            polygon.points.append(point)
        self.publisher_.publish(polygon)
        


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()