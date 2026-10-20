import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage, ControllerStatus, ODriveStatus


from gim_pkg import CONSTANTS
import math
from telemetry_interface.msg import GIM


class GIMNode(Node):



    

    def __init__(self):
        super().__init__('gim_node')
        self.e_stop = False
        self.control_vals = []

        
        self.joystick_vals = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10) # Joystick input command

        self.steering_values = self.create_publisher(GIM, 'swerve_controls', 1)
        self.timer = self.create_timer(0.1, self.publish_swerve_controls)

        self.declare_parameter('max_chassis_vel', 1.0) # Max velocity command for the chassis in m/s
        self.declare_parameter('max_chassis_ang_vel', 0.5) # Max angular command for the chassis in rad/s
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
        

    def cmd_vel_callback(self, msg):

        #checks for emergency stop button press first


        #if (e-stop button pressed):
            #if (self.e_stop is False):
                #self.e_stop = True
            #else:
                #self.e_stop = False
        if self.e_stop:
            req = AxisState.Request()
            req.axis_requested_state = 1
            future = self.client.call_async(req)
            future.add_done_callback(self.check_success)
            for publisher in self.control_vals:
                publisher.publish(ControlMessage(input_vel=0))
            return

        # Chassis forward/backward linear speed (m/s)
        self.Vc_x = msg.linear.x * self.get_parameter('max_chassis_vel').get_parameter_value().double_value

        # Chassis angular rotation speed (rad/s)
        self.omega_c = msg.angular.z * self.get_parameter('max_chassis_ang_vel').get_parameter_value().double_value

        self.kinematics()

    def kinematics(self):

        # middle right and left wheel Speeds (m/s)
        theta_right = (self.Vc_x + (self.constants['WBASE'] * self.omega_c))
        theta_left = (self.Vc_x - (self.constants['WBASE'] * self.omega_c))
        self.tank_values["Middle Left"] = theta_left
        self.tank_values["Middle Right"] = theta_right

       


        # Calculate the motion component vectors of each swerve
        i = 1
        for swerve in self.swerve_values:
            V_x = self.Vc_x - (self.omega_c * CONSTANTS.WHEEL_COORDINATES[i][1])
            V_y = self.omega_c * CONSTANTS.WHEEL_COORDINATES[i][0]
            phi = math.atan2(V_y, V_x)
            V_f = math.sqrt(V_x**2 + V_y**2)
            self.swerve_values[swerve] = [V_x, V_y, phi, V_f]
            i+=1

    def publish_swerve_controls(self):

        #Publishh speeds and turn angles for swerves and speeds for middle tank drives

        self.steering_values.publish(GIM(swerve1_controls=list(self.swerve_values["Swerve1"][2:4]),
                                         swerve2_controls=list(self.swerve_values["Swerve2"][2:4]),
                                         swerve3_controls=list(self.swerve_values["Swerve3"][2:4]),
                                         swerve4_controls=list(self.swerve_values["Swerve4"][2:4]),
                                         tank_controls=[self.tank_values.get("Middle Left", 0.0), self.tank_values.get("Middle Right", 0.0)]))


    

def main(args=None):
    rclpy.init(args=args)
    gim_node = GIMNode()
    rclpy.spin(gim_node)
    gim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()