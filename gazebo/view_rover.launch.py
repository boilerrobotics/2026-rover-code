import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))

    # Defaults to rover.urdf.xacro next to this launch file.
    # Override with:  model:=/absolute/path/to/file.urdf(.xacro)
    model = LaunchConfiguration('model')

    # xacro passes plain .urdf files through unchanged, so this works for both.
    # value_type=str stops ROS from trying to parse the XML as YAML.
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(here, 'rover.urdf.xacro'),
            description='Absolute path to the URDF or xacro file',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # Sliders for every non-fixed joint (mimic joints follow their master)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        ),
    ])