from rclpy.node import Node
import rclpy
from odrive_can.msgs import ControlMessage
from std_msgs.msg import Float64
from math import pi

class ODriveTransform(Node):
    def __init__(self):
        self.ctrl_msg_sub = self.create_subscription(ControlMessage, "cmd_in", self.convert_callback, 10)
        self.gaz_msg_pub = self.create_publisher(Float64, "cmd_out")

    def convert_callback(self, msg):
        if(msg.control_mode == 2):
            self.gaz_msg_pub.publish(Float64(msg.input_vel))
        else:
            self.gaz_msg_pub.publish(Float64(msg.input_pos) * 2 * pi)


def main():
    node = ODriveTransform()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()