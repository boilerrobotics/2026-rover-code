#!/bin/bash
cleanup() {
    trap '' INT TERM HUP EXIT          # don't re-enter, and survive our own kill
    kill -TERM -- -$$ 2>/dev/null      # polite stop for the whole group
    sleep 2
    kill -KILL -- -$$ 2>/dev/null      # force-kill anything that's still alive
}
trap cleanup EXIT
trap 'exit' INT TERM HUP

LIBGL_ALWAYS_SOFTWARE=1 ign gazebo empty.sdf &
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "ur5_robot.urdf", name: "urdf_model"'
