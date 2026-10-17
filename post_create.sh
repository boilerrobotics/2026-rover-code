#!/bin/bash
set -e

[ -d /workspaces/2026-rover-code/ros_odrive ] || git clone https://github.com/odriverobotics/ros_odrive.git /workspaces/2026-rover-code/ros_odrive
rm -rf /workspaces/2026-rover-code/ros_odrive/.git
sudo apt update
rosdep update
sudo rosdep install --from-paths 2026-rover-code --ignore-src -r -y || true