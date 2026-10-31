from rclpy.node import Node
import rclpy
from odrive_can.msg import ControlMessage
from std_msgs.msg import Float64
from math import pi
from odrive_can.srv import AxisState

from rclpy.qos import (QoSProfile, HistoryPolicy)

class ODriveTransform(Node):
    def __init__(self):
        super().__init__('transform')
        self.ctrl_msg_sub = self.create_subscription(ControlMessage, "cmd_in", self.convert_callback, QoSProfile(history=HistoryPolicy.KEEP_ALL))
        self.gaz_msg_pub = self.create_publisher(Float64, "cmd_out", 10)
        self.gaz_state_srv = self.create_service(AxisState, "state", self.srv_callback, 10)

    def convert_callback(self, msg):
        msgo = Float64()
        if(msg.control_mode == 2):
            msgo.data = msg.input_vel
            self.gaz_msg_pub.publish(msgo)
        else:
            msgo.data = msg.input_pos * (2 * pi)
            
            self.gaz_msg_pub.publish(msgo)

    def srv_callback(self, request, response):
        self.state = response.axis_state
        if (self.state != request.axis_state):
            



def main():
    rclpy.init()
    node = ODriveTransform()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()