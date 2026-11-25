#!/bin/bash
set -eo pipefail
export PYTHONPATH=/opt/carla-source/PythonAPI/carla${PYTHONPATH:+:$PYTHONPATH}
source /opt/ros/foxy/setup.bash
source /workspace/install/setup.bash
python3 - <<'PY'
import carla, pkg_resources
from agents.navigation.global_route_planner import GlobalRoutePlanner
import carla_ros_bridge.bridge
import carla_waypoint_publisher.carla_waypoint_publisher
from pnc_msgs.msg import Trajectory
print('CARLA client:', pkg_resources.get_distribution('carla').version)
print('Bridge, route publisher, and planning message imports OK')
PY
ros2 pkg executables my_pnc
ros2 launch carla_ad_demo carla_ad_demo.launch.py --show-args
for binary in /workspace/install/my_pnc/lib/my_pnc/planning_agent /workspace/install/my_pnc/lib/my_pnc/control_agent; do
    ldd "$binary"
done
