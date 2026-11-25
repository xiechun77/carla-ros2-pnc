#!/bin/bash
set -eo pipefail
check_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pnc_image="${PNC_IMAGE:-codex-carla-foxy:project-original}"
run_id="$(date +%Y%m%d-%H%M%S)-$$"
carla_check_name="carla013-loop-$run_id"
result_dir="${RESULT_DIR:-$check_dir/results-$run_id}"
mkdir -p "$result_dir"
if [[ "${FULL_ROUTE:-0}" == "1" ]]; then
    observation_sim_limit="${MAX_SIM_S:-180}"
    observation_wall_limit="${MAX_WALL_S:-300}"
else
    observation_sim_limit="${MAX_SIM_S:-12}"
    observation_wall_limit="${MAX_WALL_S:-45}"
fi
docker image inspect carlasim/carla:0.9.13 >/dev/null
docker image inspect "$pnc_image" >/dev/null
cleanup() {
    docker logs "$carla_check_name" > "$result_dir/server.log" 2>&1 || true
    docker inspect "$carla_check_name" --format '{{.State.Status}} exit={{.State.ExitCode}} oom={{.State.OOMKilled}}' > "$result_dir/server-state.txt" 2>&1 || true
    docker stop -t 10 "$carla_check_name" >/dev/null 2>&1 || true
}
docker run -d --rm --name "$carla_check_name" --label codex.task=carla-reproduction --init --gpus all --runtime=nvidia \
    --shm-size=2g -e NVIDIA_DRIVER_CAPABILITIES=all \
    carlasim/carla:0.9.13 bash ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -nosound
trap cleanup EXIT
docker run --rm --network "container:$carla_check_name" \
    --mount "type=bind,src=$check_dir/check-carla-server.py,dst=/check-carla-server.py,readonly" \
    "$pnc_image" python3 /check-carla-server.py
docker run --rm --ipc=shareable --shm-size=512m --network "container:$carla_check_name" \
    -e "SOURCE_MODIFIED=${SOURCE_MODIFIED:-0}" -e "PNC_IMAGE=$pnc_image" \
    -e "FIXED_BACKGROUND=${FIXED_BACKGROUND:-1}" -e "FULL_ROUTE=${FULL_ROUTE:-0}" \
    -e "STRICT_COMMAND_BOUNDS=${STRICT_COMMAND_BOUNDS:-1}" -e "STATIC_OBSTACLE_S=${STATIC_OBSTACLE_S:-0}" \
    -e "MAX_SIM_S=$observation_sim_limit" -e "MAX_WALL_S=$observation_wall_limit" \
    --mount "type=bind,src=$check_dir/check-closed-loop.py,dst=/check-closed-loop.py,readonly" \
    --mount "type=bind,src=$result_dir,dst=/results" \
    "$pnc_image" bash -lc \
    'source /opt/ros/foxy/setup.bash; source /workspace/install/setup.bash; export PYTHONPATH=/opt/carla-source/PythonAPI/carla:$PYTHONPATH; python3 /check-closed-loop.py'
