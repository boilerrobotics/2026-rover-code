import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point32
from odrive_can.srv import AxisState
import time

from rclpy.executors import ExternalShutdownException 
from odrive_can.msg import ControlMessage, ControllerStatus, ODriveStatus

import math
from odrive_can.msg import ControlMessage

class JointManager(Node):

    def __init__(self):
        super().__init__('joint_manager_node')

        self.axis_clients = []
        self.axis_state_subscribers = []
        self.axis_control_publishers = []
        self.state_request_timeout = 3.0
        self.watchlist_timer = [None] * 10
        self.declare_parameter('max_arm_ang_vel', 1.0) # Max angular command for the arm in rad/s
        self.declare_parameter('e-stop', False)

        self.controller_input = self.create_subscription(Point32, 'JointControllerMessage', self.controller_input_callback, 10)

        for i in range(10, 14):
            self.axis_clients.append(self.create_client(ControllerStatus, f'/odrive/axis{i}/request_axis_state'))
        for i in range(10, 14):
            self.axis_state_subscribers.append(self.create_subscription(ControllerStatus, f'/odrive/axis{i}/axis_state', lambda msg, motor=i: self.axis_state_callback(msg, motor), 10))
        for i in range(10, 14):
            self.axis_control_publishers.append(self.create_publisher(ControlMessage, f'/odrive_axis{i}/control_message', 10))

    def controller_input_callback(self, msg: Point32):
        pass

    def axis_state_callback(self, msg: ControllerStatus, motor: int):
        pass

    def publish_control_message(self, motor: int, control_msg: ControlMessage):
        pass

    def req_state(self, state):
        if self.fatal_error:
            return
        self.done = False
        self.currentreq = state
        self.get_logger().info(f"Requesting state {state} from all ODrive axes")
        
        for i, client in enumerate(self.clients):
            if not client.service_is_ready():
                self.fatal_shutdown(f"Odrive axis {i}: state service is unavailable")
                return
            req = AxisState.Request()
            req.axis_requested_state = state
            future = self.client.call_async(req)
            future.add_done_callback(lambda future, axis = i, state = state:
                                     self.check_success(future, axis, state))

    def check_success(self, future, axis, state):
        if (future.result().procedure_result == 0 or future.result().procedure_result == 1):
            self.get_logger().info(f'Result received successfully')
            if (future.result().procedure_result == 0):
                self.count += 1
            if (future.result().procedure_result == 1):
                self.watchlist[axis] = True
                self.watchlist_timer[axis] = time.time()
        else:
            raise ValueError(f"Failed to set state for axis {axis}")

        if (self.count == 10):
            self.done = True
            self.count = 0
            self.get_logger().info(f"All axis states set to {state}")