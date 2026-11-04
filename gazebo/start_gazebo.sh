#!/bin/bash
cleanup() {
    trap '' INT TERM HUP EXIT          # don't re-enter, and survive our own kill
    kill -TERM -- -$$ 2>/dev/null      # polite stop for the whole group
    sleep 2
    kill -KILL -- -$$ 2>/dev/null      # force-kill anything that's still alive
}
trap cleanup EXIT
trap 'exit' INT TERM HUP

source ../../../opt/ros/humble/setup.bash
source ../ros_odrive/odrive_node/install/setup.bash
source install/setup.bash
source ../src/gim_pkg/install/setup.bash
source ../ros_odrive/odrive_node/install/setup.bash
source ../base-station/src/install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 ign gazebo empty.sdf &
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "rover.urdf", name: "urdf_model"' &
ros2 launch view_rover.launch.py &
ros2 launch odrive_gazebo transforms.launch.py &
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/rover_bridge.yaml &
ros2 run gim_pkg gim_node &
ros2 launch joystick control_rover_launch.yml