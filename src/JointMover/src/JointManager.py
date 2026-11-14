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
        if (self.e_stop is True and e-stop button not pressed):
            return
        #Needs E-stop button input to be properly integrated
        if (e-stop button pressed):
            if (self.e_stop is False):
                self.e_stop = True
                self.state_request_timeout = 1.0
                self.req_state(1)
                return
                # Should probably add a timer to prevent accidentally disabling the emergency stop for a short period
            else:
                self.e_stop = False
                self.state_request_timeout = 3.0
                return
        else:
            message = ControlMessage()
            motor_position = msg.x / 360
            message.position = motor_position
            self.publish_control_message(10, message)

    def req_state(self, state):
        if self.fatal_error:
            return
        self.done = False
        self.currentreq = state
        self.pending_axes = set(range(10))
        self.get_logger().info(f"Requesting state {state} from all ODrive axes")

        self.state_timeout_timer = self.create_timer (self.state_request_timeout, self_state_request_timoout)
        for i in range(10):
            self.expected_states[i] = state
        
        for i, client in enumerate(self.clients):
            if not client.service_is_ready():
                self.fatal_shutdown(f"Odrive axis {i}: state service is unavailable")
                return
            req = AxisState.Request()
            req.axis_requested_state = state
            future = self.client.call_async(req)
            future.add_done_callback(lambda future, axis = i, req_state = state: 
                                     self.check_success(future, axis, req_state))



    def check_success(self, future, axis,):
        if (future.result().procedure_result == 0 or future.result().procedure_result == 1):
            self.get_logger().info(f'Result received successfully')
            if (future.result().procedure_result == 0):
                self.count += 1
            if (future.result().procedure_result == 1):
                self.watchlist[axis] = True
                self.watchlist_timer[axis] = time.time()

        if (self.count == 10):
            self.done = True
            self.count = 0
            self.currentreq = -1
            self.get_logger().info(f"All axis states set to {self.currentreq}")

    def axis_state_callback(self, msg, motor):
        if (self.e_stop):
            return
        axis_state = msg.axis_state
        if (axis_state != 1 and axis_state != 8):
            raise ValueError(f"Odrive axis {motor}: unexpected axis state {axis_state}")
        if (axis_state != self.axis_states[motor]):
            self.watchlist[motor] = False;
            self.watchlist_timer[motor] = None;
            self.axis_states[motor] = axis_state
            self.count+= 1
        if self.watchlist_timer[motor] is not None and time.time() - self.watchlist_timer[motor] > self.watchlist_timeout:
            raise ValueError(f"Odrive axis {motor}: watchlist timeout exceeded")
        if (self.count == 10):
            self.done = True
            self.count = 0
            self.currentreq = -1
            self.get_logger().info(f"All axis states set to {self.currentreq}")
