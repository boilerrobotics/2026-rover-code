#!/bin/bash
trap 'pkill -P $$' EXIT INT HUP TERM
source ../../../opt/ros/humble/setup.bash
source ../ros_odrive/odrive_node/install/setup.bash
#colcon build --packages-select odrive_gazebo
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 ign gazebo empty.sdf &
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "rover.urdf", name: "urdf_model"' &
ros2 launch view_rover.launch.py &
ros2 launch odrive_gazebo transforms.launch.py &
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/rover_bridge.yaml


