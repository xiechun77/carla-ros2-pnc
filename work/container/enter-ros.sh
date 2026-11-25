#!/bin/bash
set -eo pipefail
mapfile -t learning_demo_ids < <(docker ps --filter label=codex.carla.demo=1 --format '{{.ID}}')
if [[ ${#learning_demo_ids[@]} -ne 1 ]]; then
    printf '%s\n' '请先运行 run-demo.sh；需要恰好一个演示容器。'
    exit 1
fi
exec docker exec -it "${learning_demo_ids[0]}" bash -lc \
    'source /opt/ros/foxy/setup.bash; source /workspace/install/setup.bash; export PYTHONPATH=/opt/carla-source/PythonAPI/carla:${PYTHONPATH:-}; exec bash --norc -i' 
