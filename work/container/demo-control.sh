#!/bin/bash
set -eo pipefail
case "${1:-}" in
    play) command_code=0 ;;
    pause) command_code=1 ;;
    step) command_code=2 ;;
    *) printf '%s\n' '用法：bash demo-control.sh pause|step|play'; exit 2 ;;
esac
mapfile -t demo_ids < <(docker ps --filter label=codex.carla.demo=1 --format '{{.ID}}')
if [[ ${#demo_ids[@]} -ne 1 ]]; then
    printf '%s\n' '需要恰好一个由 run-demo.sh 启动的演示。'
    exit 1
fi
docker exec "${demo_ids[0]}" bash -lc \
    'source /opt/ros/foxy/setup.bash; source /workspace/install/setup.bash; ros2 topic pub --once /carla/control carla_msgs/msg/CarlaControl "{command: $1}"' bash "$command_code"
