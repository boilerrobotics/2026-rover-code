import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='left_front_steer',
            remappings=[
                ('cmd_in', '/odrive_axis0/control_message'),
                ('cmd_out', '/rover/left_front/steer/cmd_pos'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='left_rear_steer',
            remappings=[
                ('cmd_in', '/odrive_axis1/control_message'),
                ('cmd_out', '/rover/left_rear/steer/cmd_pos'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='right_front_steer',
            remappings=[
                ('cmd_in', '/odrive_axis2/control_message'),
                ('cmd_out', '/rover/right_front/steer/cmd_pos'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='right_rear_steer',
            remappings=[
                ('cmd_in', '/odrive_axis3/control_message'),
                ('cmd_out', '/rover/right_rear/steer/cmd_pos'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='left_front_wheel',
            remappings=[
                ('cmd_in', '/odrive_axis4/control_message'),
                ('cmd_out', '/rover/left_front/wheel/cmd_vel'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='left_rear_wheel',
            remappings=[
                ('cmd_in', '/odrive_axis5/control_message'),
                ('cmd_out', '/rover/left_rear/wheel/cmd_vel'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='right_front_wheel',
            remappings=[
                ('cmd_in', '/odrive_axis6/control_message'),
                ('cmd_out', '/rover/right_front/wheel/cmd_vel'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='right_rear_wheel',
            remappings=[
                ('cmd_in', '/odrive_axis7/control_message'),
                ('cmd_out', '/rover/right_rear/wheel/cmd_vel'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='left_middle_wheel',
            remappings=[
                ('cmd_in', '/odrive_axis8/control_message'),
                ('cmd_out', '/rover/left_middle/wheel/cmd_vel'),
            ]
        ),
        Node(
            package='odrive_gazebo',
            executable='transformer',
            name='right_middle_wheel',
            remappings=[
                ('cmd_in', '/odrive_axis9/control_message'),
                ('cmd_out', '/rover/right_middle/wheel/cmd_vel'),
            ]
        ),
    ])