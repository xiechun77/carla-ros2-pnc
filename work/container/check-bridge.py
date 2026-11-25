import json
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rosgraph_msgs.msg import Clock
from carla_msgs.msg import CarlaStatus, CarlaWorldInfo

rclpy.init()
node = rclpy.create_node('reproduction_bridge_check')
clocks = []
statuses = []
maps = []
subscriptions = [
    node.create_subscription(Clock, '/clock', lambda m: clocks.append(m.clock.sec + m.clock.nanosec / 1e9), 10),
    node.create_subscription(CarlaStatus, '/carla/status', lambda m: statuses.append(m.frame), 10),
    node.create_subscription(CarlaWorldInfo, '/carla/world_info', lambda m: maps.append(m.map_name), 10),
]
with open('/tmp/reproduction-bridge.log', 'w') as log:
    process = subprocess.Popen([
        'ros2', 'launch', 'carla_ros_bridge', 'carla_ros_bridge.launch.py',
        'host:=127.0.0.1', 'port:=2000', 'town:=Town01',
        'synchronous_mode:=True', 'fixed_delta_seconds:=0.05',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    success = False
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(clocks) >= 5 and len(statuses) >= 5 and maps:
                success = clocks[-1] > clocks[0] and statuses[-1] > statuses[0]
                if success:
                    break
        print(json.dumps({'clock_samples': len(clocks), 'status_samples': len(statuses),
            'clock_first_last': [clocks[0], clocks[-1]] if clocks else [],
            'frames_first_last': [statuses[0], statuses[-1]] if statuses else [],
            'maps': maps, 'passed': success}, indent=2))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
    print(Path('/tmp/reproduction-bridge.log').read_text())
    if not success:
        raise SystemExit(1)
