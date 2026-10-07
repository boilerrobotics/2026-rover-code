import rclpy
from rclpy.node import Node

from odrive_can.msg import ODriveStatus
from odrive_can.msg import ControllerStatus
from telemetry_interface.msg import TelemetryData
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class TelemetryNode(Node):
    odrive_telemetry_data = { 
        # The swerve modules plus the 2 regular odrive axes. 
        # The swerve modules are the first 8 axes, and the regular odrive axes are the last 2 axes.
        # The angular motors are the even numbered axes, and the linear motors are the odd numbered axes.

        # Swerve Module 1
        'odrive_axis0': {'can_id': 0, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        'odrive_axis1': {'can_id': 1, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        # Swerve Module 2
        'odrive_axis2': {'can_id': 2, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        'odrive_axis3': {'can_id': 3, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        # Swerve Module 3
        'odrive_axis4': {'can_id': 4, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        'odrive_axis5': {'can_id': 5, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        # Swerve Module 4
        'odrive_axis6': {'can_id': 6, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        'odrive_axis7': {'can_id': 7, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        # ODrive Axis 9 and 10
        'odrive_axis8': {'can_id': 8, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
        'odrive_axis9': {'can_id': 9, 'bus_voltage': 0.0, 'bus_current': 0.0,
                      'active_errors': 0, 'axis_state': 0,
                      'pos_estimate': 0.0, 'vel_estimate': 0.0},
    }
    
    def __init__(self):
        # Initialize the node and create publishers and subscribers
        self.swerve_controlstatus_subscribers_list = []
        self.swerve_odrivestatus_subscribers_list = []
        self.odrive_subscribers_list = []
        super().__init__('telemetry_node')

        self.telemetry_publisher = self.create_publisher(TelemetryData, 'telemetry_node', 10)

        self.timer = self.create_timer(0.1, self.publish_telemetry)

        # For each swerve motor create a subscriber for the controller status and odrive status topics
        for i in range(8):
            self.swerve_controlstatus_subscribers_list.append(self.create_subscription(
                ControllerStatus,
                f'/odrive_axis{i}/controller_status', # Topic name is motor number then controller status
                lambda msg: self.controller_status_callback(msg,f'odrive_axis{i}'),

                #QoS was giving me an error that it needed depth so I gave it a depth of 1
                # I dont know what depth does but it seems to work so I guess its fine for now
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            ))

            self.swerve_odrivestatus_subscribers_list.append(self.create_subscription(
                ODriveStatus,
                f'/odrive_axis{i}/odrive_status', # Topic name is motor number then odrive status which is different from swerve status
                lambda msg: self.odrive_status_callback(msg,f'odrive_axis{i}'),
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            ))

        # for each odrive motor create a subscriber for the controller status and odrive status topics
        # I dont know if this needs to be seperate from the swerve motors but I seperated them just cuz
        for i in range(8,10):
            self.odrive_subscribers_list.append(self.create_subscription(
                ControllerStatus,
                f'/odrive_axis{i}/controller_status',
                lambda msg: self.controller_status_callback(msg,f'odrive_axis{i}'),
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            ))
            self.odrive_subscribers_list.append(self.create_subscription(
                ODriveStatus,
                f'/odrive_axis{i}/odrive_status',
                lambda msg: self.odrive_status_callback(msg,f'odrive_axis{i}'),

                
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            ))
            

    # Callback functions for the ODriveStatus subscribers
    def odrive_status_callback(self,msg, key):
        self.swerve_telemetry_data = self.odrive_telemetry_data[key].update({'bus_voltage': msg.bus_voltage,
         'bus_current': msg.bus_current})

    # Callback function for the ControllerStatus subscribers
    def controller_status_callback(self, msg, key):
        self.odrive_telemetry_data[key].update({'active_errors': msg.active_errors,
         'axis_state': msg.axis_state, 'pos_estimate': msg.pos_estimate,
         'vel_estimate': msg.vel_estimate})

    # Function to publish the telemetry data
    def publish_telemetry(self):
        voltage_sum = 0
        current_sum = 0
        active_errors = []
        estimated_pos = []
        estimated_vel = []

        # Loop through the odrive_telemetry_data dictionary and sum the bus voltage and current, and append the active errors, estimated position, and estimated velocity to their respective lists
        for i in TelemetryNode.odrive_telemetry_data.values():

            # for the estimated poses and velocities well have to make sure that 
            # we isolate the position motor and the velocity motor from the disctionary
            # I did this by just assuming that the even can_ids are the angular motors
            # and the odd can_ids are the linear motors.

            # Check if things are None because we don't want things that are none?
           
            voltage_sum += i['bus_voltage']
            current_sum += i['bus_current']
            active_errors.append(int(i['active_errors']))

            # making sure that pos_estimate gets 4 float values everytime like it so desperately needs
            if i['can_id'] < 8 and i['can_id'] % 2 == 0:
                estimated_pos.append(float(i['pos_estimate'])) # Change to float because error if pos_estimate is not float32
        
            if i['can_id'] < 8 and i['can_id'] % 2 == 1:
                estimated_vel.append(float(i['vel_estimate']))
            
        
           
        # Testing what telemetry data looks like and right now everything is empty :()
        #print(f"Publishing telemetry data: average_voltage={voltage_sum/8}\n, average_current={current_sum/8}\n, estimated_pos={estimated_pos}\n, estimated_vel={estimated_vel}\n, active_errors={active_errors}\n")
        #print("Length of Estimated Pos: ",len(estimated_pos), "\nLength of Estimated Vel: ", len(estimated_vel))

        self.telemetry_publisher.publish(TelemetryData(
            average_voltage=voltage_sum/8, 
            average_current=current_sum/8, 
            estimated_pos=estimated_pos, 
            estimated_vel=estimated_vel, 
            active_errors=active_errors))
            
# Apparently this main function is necessary because ros2 looks for the function name in setup.py and __main__ is not the function name for main
def main(args=None):
    rclpy.init()
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()