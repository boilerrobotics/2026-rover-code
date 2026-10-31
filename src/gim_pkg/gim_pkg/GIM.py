import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from odrive_can.srv import AxisState
import time

from odrive_can.msg import ControlMessage, ControllerStatus, ODriveStatus


from gim_pkg import CONSTANTS
import math
from odrive_can.msg import ControlMessage


class GIMNode(Node):

    def __init__(self):
        super().__init__('gim_node')
        self.e_stop = False
        self.control_vals = []
        self.done = True
        self.count = 0
        self.currentreq = -1

        
        self.joystick_vals = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10) # Joystick input command

        self.control_vals = []

        self.axis_clients = []

        for i in range(10):
            self.control_vals.append(self.create_publisher(ControlMessage, f'/odrive_axis{i}/control_message', 10))

        
        for i in range(10):
            self.axis_clients.append(self.create_client(AxisState, f'/odrive/axis{i}/request_axis_state'))

        self.declare_parameter('max_chassis_vel', 1.0) # Max velocity command for the chassis in m/s
        self.declare_parameter('max_chassis_ang_vel', 1.0) # Max angular command for the chassis in rad/s
        self.declare_parameter('e-stop', False) # Max velocity command for the swerve in m/s
        self.Vc_x = 0.0  # Chassis velocity in the x direction (m/s)
        self.omega_c = 0.0  # Chassis angular velocity (rad/s)
        self.constants = {"WRADIUS": CONSTANTS.WRADIUS, "WBASE": CONSTANTS.WBASE}

        """[Component Velocity X (V_x), 
            Component Velocity Y (V_y),
            Steering Angle (phi),
            Drive Motor Speed (V_f)]
        """
        
        self.swerve_values = {
            "Swerve1": [0.0, 0.0, 0.0, 0.0], 
            "Swerve2": [0.0, 0.0, 0.0, 0.0], 
            "Swerve3": [0.0, 0.0, 0.0, 0.0], 
            "Swerve4": [0.0, 0.0, 0.0, 0.0]
        }
        self.tank_values = {"Middle Left": 0.0, "Middle Right" : 0.0}

        """with open("/home/kamil-poreba/Documents/BRC/2026-rover-code/src/gim_pkg/CONSTANTS.txt", "r") as f:
            lines = f.readlines()
            for l in lines:
                key, value = l.strip().split(",")
                self.constants[key] = float(value)"""

        print(f"Wheel radius: {self.constants['WRADIUS']}, Wheel base: {self.constants['WBASE']}")
        

    def req_state(self, state):
        if (self.clients[0].axis_requested_state == state):
            self.get_logger().info(f"Axis state already set to {state}")
            return
        self.done = False
        self.currentreq = state
        for i in range(10):
            req = AxisState.Request()
            req.axis_requested_state = state
            future = self.clients[i].call_async(req)
            future.add_done_callback(self.check_success)

    def check_success(self, future):
        if (future.result().procedure_result == 0):
            self.get_logger().info(f'Result received successfully')
            self.count += 1
        else:
            startTime = time.time()
            
            while (AxisState.Request().axis_requested_state != self.currentreq):
                newTime = time.time()
                if (req.axis_requested_state != 1 or req.axis_requested_state != 8 or (newTime - startTime) > 1):
                    raise Exception(f"Timer expired or axis state error: current state {req.axis_requested_state}")
            self.count+=1
        if (self.count == 10):
            self.done = True
            self.count = 0
            self.currentreq = -1
            self.get_logger().info(f"All axis states set to {self.currentreq}")
        


    def cmd_vel_callback(self, msg):

        #checks for emergency stop button press first


        #if (e-stop button pressed):
            #if (self.e_stop is False):
                #self.e_stop = True
            #else:
                #self.e_stop = False

        if (self.get_parameter('e-stop').get_parameter_value().bool_value):
            self.req_state(1, True)
            for publisher in self.control_vals:
                publisher.publish(ControlMessage(input_vel=0))
            return
        if (self.done is True):
            if (msg.angular.y == 1):
                self.req_state(1)
            elif (msg.angular.x == 1):
                self.req_state(8)

        # Chassis forward/backward linear speed (m/s)
        self.Vc_x = msg.linear.x * self.get_parameter('max_chassis_vel').get_parameter_value().double_value

        # Chassis angular rotation speed (rad/s)
        self.omega_c = msg.angular.z * self.get_parameter('max_chassis_ang_vel').get_parameter_value().double_value

        self.kinematics()

         #Publishh speeds and turn angles for swerves and speeds for middle tank drives
        self.control_vals[0].publish(ControlMessage(input_pos=self.swerve_values.get("Swerve1")[2]))
        self.control_vals[1].publish(ControlMessage(control_mode = 2,input_vel=self.swerve_values["Swerve1"][3]))
        self.control_vals[2].publish(ControlMessage(input_pos=self.swerve_values["Swerve2"][2]))
        self.control_vals[3].publish(ControlMessage(control_mode = 2,input_vel=self.swerve_values["Swerve2"][3]))
        self.control_vals[4].publish(ControlMessage(input_pos=self.swerve_values["Swerve3"][2]))
        self.control_vals[5].publish(ControlMessage(control_mode = 2,input_vel=self.swerve_values["Swerve3"][3]))
        self.control_vals[6].publish(ControlMessage(input_pos=self.swerve_values["Swerve4"][2]))
        self.control_vals[7].publish(ControlMessage(control_mode = 2,input_vel=self.swerve_values["Swerve4"][3]))
        self.control_vals[8].publish(ControlMessage(control_mode = 2,input_vel=self.tank_values["Middle Left"]))
        self.control_vals[9].publish(ControlMessage(control_mode = 2,input_vel=self.tank_values["Middle Right"]))
        

    def kinematics(self):

        # middle right and left wheel Speeds (m/s)
        theta_right = (self.Vc_x + (self.constants['WBASE'] * self.omega_c))
        theta_left = (self.Vc_x - (self.constants['WBASE'] * self.omega_c)) 
        self.tank_values["Middle Left"] = theta_left / CONSTANTS.WRADIUS
        self.tank_values["Middle Right"] = theta_right / CONSTANTS.WRADIUS

        # Calculate the motion component vectors of each swerve
        i = 1
        for swerve in self.swerve_values:
            V_x = self.Vc_x - (self.omega_c * CONSTANTS.WHEEL_COORDINATES[i][1])
            V_y = self.omega_c * CONSTANTS.WHEEL_COORDINATES[i][0]
            phi = math.atan2(V_y, V_x) / (2 * math.pi)
            V_f = math.sqrt(V_x**2 + V_y**2) / (CONSTANTS.WRADIUS)
            if phi > .25:
                phi -= .5; V_f = -V_f
            elif phi < -.25:
                phi += 0.5; V_f = -V_f
            self.swerve_values[swerve] = [V_x, V_y, phi, V_f]
            i+=1

def main():
    rclpy.init()
    gim_node = GIMNode()
    rclpy.spin(gim_node)
    gim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()