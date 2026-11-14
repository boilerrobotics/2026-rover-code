# Intended to be run from the base directory
# after compilation

#!/usr/bin/bash

source install/setup.bash

cleanup() {
    kill $(jobs -p)
}
trap cleanup EXIT

ros2 topic pub /global_destination geometry_msgs/msg/Point32 '{x: 200.0, y: 200.0, z: 0.0}' &
PID1=$!
ros2 topic pub /current_position geometry_msgs/msg/Point32 '{x: 10.0, y: 20.0, z: 0.0}' &
PID2=$!

# Should map to coordinate (17, 32)
ros2 topic pub /obstacle_locations obstacle_interfaces/msg/ObstacleLocations '{locations: [{x: 12.0, y: -7.0, z: 0.0}]}' &
PID3=$!

# Should map to coordinate (14, 30)
ros2 topic pub /aruco_locations aruco_interfaces/msg/ArucoLocations '{markers: [{x: 10.0, y: -4.0, z: 0.0}]}' &
PID3=$!

# Facing positive y-direction
ros2 topic pub /current_orientation geometry_msgs/msg/Quaternion '{x: 0.0, y: 1.0, z: 0.0, w: 1.0}' &
PID4=$!

wait