#!/bin/bash
set -eo pipefail
check_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
carla_check_name="carla013-check-$(date +%s)-$$"
docker image inspect carlasim/carla:0.9.13 >/dev/null
docker image inspect codex-carla-foxy:project-original >/dev/null
cleanup() {
    docker logs "$carla_check_name" > "$check_dir/carla-check-server.log" 2>&1 || true
    docker stop -t 10 "$carla_check_name" >/dev/null 2>&1 || true
}
docker run -d --rm --name "$carla_check_name" --init --gpus all --runtime=nvidia \
    --shm-size=2g -e NVIDIA_DRIVER_CAPABILITIES=all \
    carlasim/carla:0.9.13 bash ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -nosound
trap cleanup EXIT
docker run --rm --network "container:$carla_check_name" \
    --mount "type=bind,src=$check_dir/check-carla-server.py,dst=/check-carla-server.py,readonly" \
    codex-carla-foxy:project-original python3 /check-carla-server.py
docker run --rm --network "container:$carla_check_name" \
    --mount "type=bind,src=$check_dir/check-bridge.py,dst=/check-bridge.py,readonly" \
    codex-carla-foxy:project-original bash -lc \
    'source /opt/ros/foxy/setup.bash; source /workspace/install/setup.bash; export PYTHONPATH=/opt/carla-source/PythonAPI/carla:$PYTHONPATH; python3 /check-bridge.py'
