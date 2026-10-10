import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist





class GIMNode(Node):



    

    def __init__(self):
        super().__init__('gim_node')

        #self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10) # Joystick input command

        self.declare_parameter('max_chassis_vel', 1.0) # Max velocity command for the chassis in m/s
        self.declare_parameter('max_chassis_ang_vel', 0.5) # Max angular command for the chassis in rad/s
        self.Vc_x = 0.0  # Chassis velocity in the x direction (m/s)
        self.omega_c = 0.0  # Chassis angular velocity (rad/s)
        self.constants = {}
        with open("/workspaces/2026-rover-code/src/GIM_pkg/CONSTANTS.txt", "r") as f:
            lines = f.readlines()
            for l in lines:
                key, value = l.strip().split(",")
                self.constants[key] = float(value)

        print(f"Wheel radius: {self.constants['WRADIUS']}, Wheel base: {self.constants['WBASE']}")
        

   # def cmd_vel_callback(self, msg):

      #  self.Vc_x = msg.linear.x * self.get_parameter('max_chassis_vel').get_parameter_value().double_value
       # self.omega_c = msg.angular.z * self.get_parameter('max_chassis_ang_vel').get_parameter_value().double_value


def main(args=None):
    rclpy.init(args=args)
    gim_node = GIMNode()
    rclpy.spin(gim_node)
    gim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()