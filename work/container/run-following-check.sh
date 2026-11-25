#!/bin/bash
set -eo pipefail
container_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd -- "$container_dir/../.." && pwd)"
if [[ -n "$(docker ps -q --filter label=codex.carla.demo=1)" ]]; then
  echo '请先结束现有演示，再启动跟车诊断。' >&2; exit 1
fi
mode="${1:---gui}"
if [[ "$mode" != --gui && "$mode" != --headless ]]; then echo '用法: run-following-check.sh [--gui|--headless]';exit 2;fi
run_id="$(date +%Y%m%d-%H%M%S)-$$"
server="carla-follow-server-$run_id"
runner="carla-follow-check-$run_id"
results="$root_dir/experiments/following/results/$run_id"
mkdir -p "$results"
cleanup(){
 docker logs "$runner" > "$results/runner-docker.log" 2>&1 || true
 docker stop -t 30 "$runner" >/dev/null 2>&1 || true
 docker logs "$server" > "$results/server.log" 2>&1 || true
 docker stop -t 10 "$server" >/dev/null 2>&1 || true
}
trap cleanup EXIT
ui_args=()
if [[ "$mode" == --gui ]];then
 : "${DISPLAY:?请从 Ubuntu 图形桌面终端运行}"; : "${XAUTHORITY:?缺少 XAUTHORITY}"
 ui_args=(--ipc=host --gpus all --runtime=nvidia -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display -e DISPLAY -e XAUTHORITY=/tmp/carla-xauthority -e FOLLOW_GUI=1 -e QT_X11_NO_MITSHM=1 -e SDL_VIDEODRIVER=x11 -e SDL_AUDIODRIVER=dummy --mount "type=bind,src=$XAUTHORITY,dst=/tmp/carla-xauthority,readonly" --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly)
fi
printf '本次记录: %s\n' "$results"
docker run -d --rm --init --name "$server" --label codex.task=carla-reproduction --gpus all --runtime=nvidia --shm-size=2g -e NVIDIA_DRIVER_CAPABILITIES=all carlasim/carla:0.9.13 bash ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -nosound
docker run --rm --network "container:$server" --mount "type=bind,src=$container_dir/check-carla-server.py,dst=/check.py,readonly" codex-carla-foxy:gui python3 /check.py > "$results/readiness.log"
# Persist precisely the scene used, even if the user edits the template later.
cp "$root_dir/experiments/following/scenario.json" "$results/scenario.json"
cp "$root_dir/experiments/following/objects.json" "$results/objects.json"
find "$root_dir/work/controller-dev/src/my_planning_and_control" -type f -print0 | sort -z | xargs -0 sha256sum > "$results/source-sha256.txt"
docker run --rm --init --name "$runner" --label codex.carla.demo=1 --label codex.task=carla-reproduction --network "container:$server" --user "$(id -u):$(id -g)" --workdir /results -e HOME=/tmp -e PNC_DEBUG_DIR=/results -e PYTHONPATH=/opt/carla-source/PythonAPI/carla "${ui_args[@]}" \
 --mount "type=bind,src=$root_dir/work/controller-dev,dst=/learning,readonly" \
 --mount "type=bind,src=$results,dst=/results" \
 --mount "type=bind,src=$results,dst=/scenario,readonly" \
 --mount "type=bind,src=$container_dir/carla_local_path.rviz,dst=/local-path.rviz,readonly" \
 --mount "type=bind,src=$container_dir/following-check.py,dst=/following-check.py,readonly" \
 codex-carla-foxy:gui bash -lc 'set -e; source /opt/ros/foxy/setup.bash; source /workspace/install/setup.bash; source /learning/install/local_setup.bash; test "$(ros2 pkg prefix my_pnc)" = /learning/install/my_pnc; python3 -u /following-check.py' | tee "$results/console.log"
echo "诊断结束，请查看 $results/summary.json 和 samples.csv"
