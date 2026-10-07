import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import Image, CameraInfo
from aruco_interfaces.msg import ArucoLocations

import numpy as np
from cv_bridge import CvBridge

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.locations_pub = self.create_publisher(
            ArucoLocations,
            "/aruco_locations",
            qos_profile_system_default
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/zed/zed_node/left/color/rect/camera_info",
            self.camera_info_callback,
            qos_profile_system_default
        )

        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            # TESTING: Matches bag image; NOT ARUCO
            cv2.aruco.DICT_APRILTAG_36h11
        )

        self.bridge = CvBridge()

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

        self.destroy_subscription(self.camera_info_sub)
        self.initalize_depth_callbacks()

    # Must wait for camera information to be passed
    # through in order to calculate relative position
    def initalize_depth_callbacks(self):
        self.depth_sub = Subscriber(
            self,
            Image,
            "/zed/zed_node/depth/depth_registered",
            qos_profile_system_default
        )

        self.image_sub = Subscriber(
            self,
            Image,
            "/zed/zed_node/left/color/rect/image",
            qos_profile_system_default
        )

        # Synchronizes depth and image maps
        # slop: acceptable time gap in seconds
        self.time_sync = ApproximateTimeSynchronizer([self.depth_sub, self.image_sub],
                                                    queue_size=10, slop=0.5)
        self.time_sync.registerCallback(self.depth_callback)

    # Converts from u, v coordinates in image to 3d
    # coordinates relative to left camera in rep-103
    def parse_point(self, u, v, depth):
        point = Point32()
        point.x = float(depth)
        point.y = (self.cx - u) * depth / self.fx
        point.z = (self.cy - v) * depth / self.fy
        return point

    def to_point(self, arr):
        point = Point32()
        point.x = float(arr[0])
        point.y = float(arr[1])
        point.z = 0.0
        return point
        
    def depth_callback(self, depth: Image, image: Image):
        cv2_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')

        gray_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
        boxes, ids, _ = cv2.aruco.detectMarkers(
            gray_image,
            self.aruco_dict,
            parameters=self.aruco_params,
        )

        if ids is not None:

            aruco_markers = ArucoLocations()

            for box in boxes:
                corners = box[0]
                cx = round(np.mean(corners[:, 0]))
                cy = round(np.mean(corners[:, 1]))
                cd = depth_image[cy, cx]

                if (np.isnan(cd)):
                    # DEBUGGING: Often camera cannot determine depth
                    # self.get_logger().info("ArUco marker found, but no depth provided")
                    continue

                bounding_box = Polygon()
                for corner in corners:
                    bounding_box.points.append(self.to_point(corner))

                marker = self.parse_point(cx, cy, cd)
                aruco_markers.markers.append(marker)
                aruco_markers.bounding_boxes.append(bounding_box)
                aruco_markers.centers.append(self.to_point([cx, cy]))

            if len(aruco_markers.markers) > 0:
                # DEBUGGING: Writes images to file system
                # annotated_image = cv2.aruco.drawDetectedMarkers(cv2_image, boxes, ids)
                # file_path = f'{image.header.stamp.sec}_{image.header.stamp.nanosec}.jpg'
                # cv2.imwrite(file_path, annotated_image)

                self.locations_pub.publish(aruco_markers)
                self.get_logger().info(f"Updated ArUco Locations: {aruco_markers}")

def main(args=None):
    rclpy.init(args=args)

    aruco = ArucoNode()

    rclpy.spin(aruco)

    aruco.destroy_node()
    rclpy.shutdown()