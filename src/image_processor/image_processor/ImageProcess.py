import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from id_polygon.msg import IdPolygon
from message_filters import Subscriber, ApproximateTimeSynchronizer

import cv2
import numpy as np

class ArUcoNode(Node):
    def __init__(self):
        super().__init__('image_processor')

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
        # DICT_APRILTAG_36h11 is bag tag, otherwise use DICT_4X4_50
        )

        self.aruco_parameters = (
        cv2.aruco.DetectorParameters_create()
        )

        self.subscription_image = Subscriber(
            self,
            Image,
            '/zed/zed_node/left/color/rect/image',
            qos_profile=qos_profile_sensor_data
        )

        self.subscription_depth = Subscriber(
            self,
            Image,
            '/zed/zed_node/depth/depth_registered',
            qos_profile=qos_profile_sensor_data
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.subscription_image, self.subscription_depth],
            queue_size=10,
            slop=0.05
        )

        self.sync.registerCallback(self.image_depth_callback)

        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/left/color/rect/camera_info',
            self.camera_info_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.publisher_polygon = self.create_publisher(IdPolygon, '/aruco_locations', qos_profile_system_default)

    def image_depth_callback(self, image_msg, depth_msg):

        # Convert RGB
        image = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8'
        )

        # Convert depth
        depth_image = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding='32FC1'
        )

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_parameters
        )
        if ids is not None:
            self.get_logger().info(f"Detected IDs: {ids.flatten()}")

        if ids is None:
            self.get_logger().info(f"No IDs detected, rejected markers: {len(rejected)}")
            return

        coords = np.zeros((len(ids), 4, 3))

        for i in range(len(ids)):

            marker_corners = corners[i][0]

            for j in range(4):

                u = int(round(marker_corners[j][0]))
                v = int(round(marker_corners[j][1]))

                depth_values = []

                for h in range(-10, 11):
                    for k in range(-10, 11):

                        y = v + h
                        x = u + k

                        if (
                            0 <= y < depth_image.shape[0] and
                            0 <= x < depth_image.shape[1]
                        ):

                            depth = depth_image[y, x]

                            if np.isfinite(depth) and depth > 0:
                                depth_values.append(depth)

                if depth_values:
                    coords[i][j] = [
                        u,
                        v,
                        np.median(depth_values)
                    ]
                else:
                    coords[i][j] = [
                        u,
                        v,
                        np.nan
                    ]

        self.ArUco_publisher(ids, coords)
            
    
    def camera_info_callback(self, msg):
        # Process camera info constantly
        if msg is not None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def ArUco_publisher(self, ids, coords):
        if (
            self.fx is None or
            self.fy is None or
            self.cx is None or
            self.cy is None
        ):
            return
        for i in range(len(coords)):
            message = IdPolygon()
            message.id = int(ids[i][0])
            for j in range(4):
                point = Point32()
                point.x = float(coords[i][j][2])
                point.y = float((self.cx - coords[i][j][0]) / self.fx * coords[i][j][2])
                point.z = float((self.cy - coords[i][j][1]) / self.fy * coords[i][j][2])
                message.polygon.points.append(point)
            # WARNING: Center calculated as a mean of all x/y coordinates, which may not work if only one corner is detected. Needs a better solution later.
            message.center.x = float(np.mean(coords[i][:, 0]))
            message.center.y = float(np.mean(coords[i][:, 1]))
            message.center.z = 0.00
            self.publisher_polygon.publish(message)
        


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()