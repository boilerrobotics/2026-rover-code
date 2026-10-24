#!/usr/bin/env bash
# Drive the rover forward at a given speed (m/s) in Gazebo.
#
# Usage:   ./move_rover.sh <speed_m_per_s>
# Example: ./move_rover.sh 0.5     # forward
#          ./move_rover.sh 0       # stop
#          ./move_rover.sh -0.3    # reverse
#
# Optional env vars:  WHEEL_RADIUS (default 0.10)   NS (default /rover)

set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 <speed_m_per_s>" >&2
  exit 1
fi

SPEED="$1"
RADIUS="${WHEEL_RADIUS:-0.10}"
NS="${NS:-/rover}"

# Check that the speed is a number
if ! [[ "$SPEED" =~ ^-?[0-9]*\.?[0-9]+$ ]]; then
  echo "error: speed must be a number, got '$SPEED'" >&2
  exit 1
fi

# Fortress uses `ign`; Garden and later use `gz`
if command -v ign >/dev/null 2>&1; then
  CLI=ign
  MSG=ignition.msgs.Double
elif command -v gz >/dev/null 2>&1; then
  CLI=gz
  MSG=gz.msgs.Double
else
  echo "error: neither 'ign' nor 'gz' found on PATH" >&2
  exit 1
fi

# wheel angular velocity (rad/s) = linear speed / wheel radius
OMEGA=$(awk -v v="$SPEED" -v r="$RADIUS" 'BEGIN { printf "%.6f", v / r }')

for side in left right; do
  for pos in front middle rear; do
    "$CLI" topic -t "$NS/${side}_${pos}/wheel/cmd_vel" -m "$MSG" -p "data: $OMEGA"
  done
done

echo "speed $SPEED m/s -> $OMEGA rad/s on all 6 wheels"