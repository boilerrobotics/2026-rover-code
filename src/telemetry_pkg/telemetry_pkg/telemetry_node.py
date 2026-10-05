import rclpy
from rclpy.node import Node

from telemetry_interface.msg import ODriveStatus
from telemetry_interface.msg import ControllerStatus
from telemetry_interface.msg import TelemetryData
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class TelemetryNode(Node):
    odrive_telemetry_data = { 
        # The swerve modules plus the 2 regular odrive axes. 
        # The swerve modules are the first 8 axes, and the regular odrive axes are the last 2 axes.
        # The angular motors are the even numbered axes, and the linear motors are the odd numbered axes.

        # Swerve Module 1
        'swerve_axis0': {'can_id': 0, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        'swerve_axis1': {'can_id': 1, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        # Swerve Module 2
        'swerve_axis2': {'can_id': 2, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        'swerve_axis3': {'can_id': 3, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        # Swerve Module 3
        'swerve_axis4': {'can_id': 4, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        'swerve_axis5': {'can_id': 5, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        # Swerve Module 4
        'swerve_axis6': {'can_id': 6, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        'swerve_axis7': {'can_id': 7, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        # ODrive Axis 9 and 10
        'odrive_axis8': {'can_id': 8, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
        'odrive_axis9': {'can_id': 9, 'bus_voltage': None, 'bus_current': None,
                      'active_errors': None, 'axis_state': None,
                      'pos_estimate': None, 'vel_estimate': None},
    }
    
    def __init__(self):
        self.swerve_controlstatus_subscribers_list = []
        self.swerve_odrivestatus_subscribers_list = []
        self.odrive_subscribers_list = []
        super().__init__('telemetry_node')

        self.telemetry_publisher = self.create_publisher(TelemetryData, 'telemetry_node', 10)

        self.timer = self.create_timer(0.1, self.publish_telemetry)
        for i in range(8):
            self.swerve_controlstatus_subscribers_list.append(self.create_subscription(
                ControllerStatus,
                f'/swerve_axis{i}/controller_status',
                lambda msg: self.controller_status_callback(msg,f'swerve_axis{i}'),
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            ))

            self.swerve_odrivestatus_subscribers_list.append(self.create_subscription(
                ODriveStatus,
                f'/swerve_axis{i}/swerve_status',
                lambda msg: self.odrive_status_callback(msg,f'swerve_axis{i}'),
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            ))

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
            

    def odrive_status_callback(self,msg, key):
        self.swerve_telemetry_data = self.odrive_telemetry_data[key].update({'bus_voltage': msg.bus_voltage,
         'bus_current': msg.bus_current})


    def controller_status_callback(self, msg, key):
        self.odrive_telemetry_data[key].update({'active_errors': msg.active_errors,
         'axis_state': msg.axis_state, 'pos_estimate': msg.pos_estimate,
         'vel_estimate': msg.vel_estimate})

    def publish_telemetry(self):
        voltage_sum = 0
        current_sum = 0
        active_errors = []
        estimated_pos = []
        estimated_vel = []

        
        for i in TelemetryNode.odrive_telemetry_data.values():

            # for the estimated poses and velocities well have to make sure that 
            # we isolate the position motor and the velocity motor from the disctionary
            # I did this by just assuming that the even can_ids are the angular motors
            # and the odd can_ids are the linear motors.

            if i['bus_voltage'] is not None:
                voltage_sum += i['bus_voltage']

            if i['bus_current'] is not None:
                current_sum += i['bus_current']

            if i['active_errors'] is not None:
                active_errors.append(i['active_errors'])

            # making sure that pos_estimate gets 4 float values everytime like it so desperately needs
            if i['pos_estimate'] is not None and i['can_id'] < 8 and i['can_id'] % 2 == 0:
                estimated_pos.append(float(i['pos_estimate'])) # Change to float because error if pos_estimate is not float32
        
            if i['vel_estimate'] is not None and i['can_id'] < 8 and i['can_id'] % 2 == 1:
                estimated_vel.append(float(i['vel_estimate']))
            
        
           
           
        print(f"Publishing telemetry data: average_voltage={voltage_sum/8}\n, average_current={current_sum/8}\n, estimated_pos={estimated_pos}\n, estimated_vel={estimated_vel}\n, active_errors={active_errors}\n")
        print("Length of Estimated Pos: ",len(estimated_pos), "\nLength of Estimated Vel: ", len(estimated_vel))
        self.telemetry_publisher.publish(TelemetryData(
            average_voltage=voltage_sum/8, 
            average_current=current_sum/8, 
            estimated_pos=estimated_pos, 
            estimated_vel=estimated_vel, 
            active_errors=active_errors))
            

def main(args=None):
    rclpy.init()
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()