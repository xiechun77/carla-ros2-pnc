#!/bin/bash
set -eo pipefail
demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
dev_dir="$(cd -- "$demo_dir/../controller-dev" && pwd)"
test -f "$dev_dir/install/local_setup.bash"
if [[ -n "$(docker ps -q --filter label=codex.carla.demo=1)" ]]; then
    echo "请先结束现有演示，再启动实验演示，避免多个控制器同时运行。" >&2
    exit 1
fi
demo_id="$(date +%Y%m%d-%H%M%S)-$$"
server_name="carla013-experiment-$demo_id"
demo_name="carla-ros-experiment-$demo_id"
demo_results="$demo_dir/demo-$demo_id"
mkdir -p "$demo_results"
: "${DISPLAY:?请从当前 Ubuntu 图形桌面的终端运行}"
: "${XAUTHORITY:?当前桌面没有提供 XAUTHORITY，请先检查显示会话}"
test -r "$XAUTHORITY"
docker image inspect carlasim/carla:0.9.13 >/dev/null
if ! docker image inspect codex-carla-foxy:gui >/dev/null 2>&1; then
    docker build -t codex-carla-foxy:gui -f "$demo_dir/Dockerfile.gui" "$demo_dir"
fi
cleanup() {
    if docker inspect "$demo_name" >/dev/null 2>&1; then
        docker logs "$demo_name" > "$demo_results/docker-demo.log" 2>&1 || true
    fi
    docker stop -t 12 "$demo_name" >/dev/null 2>&1 || true
    docker logs "$server_name" > "$demo_results/server.log" 2>&1 || true
    docker stop -t 10 "$server_name" >/dev/null 2>&1 || true
}
docker run -d --rm --name "$server_name" --label codex.task=carla-reproduction --init \
    --gpus all --runtime=nvidia --shm-size=2g -e NVIDIA_DRIVER_CAPABILITIES=all \
    carlasim/carla:0.9.13 bash ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -nosound
trap cleanup EXIT
docker run --rm --network "container:$server_name" \
    --mount "type=bind,src=$demo_dir/check-carla-server.py,dst=/check-carla-server.py,readonly" \
    codex-carla-foxy:project-original python3 /check-carla-server.py > "$demo_results/readiness.log" 2>&1
printf '%s\n' '正在打开 RViz 和车辆画面。关闭 RViz 或在此终端按 Ctrl+C 可结束演示。'
docker run --rm --name "$demo_name" --label codex.carla.demo=1 --label codex.task=carla-reproduction \
    --init --ipc=host --network "container:$server_name" --gpus all --runtime=nvidia \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display -e DISPLAY \
    -e SDL_AUDIODRIVER=dummy -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 -e SDL_VIDEODRIVER=x11 -e XAUTHORITY=/tmp/carla-xauthority -e PNC_DEBUG_DIR=/demo-results -e PNC_PLANNER_TYPE="${PNC_PLANNER_TYPE:-em}" -e PNC_CONTROLLER_TYPE="${PNC_CONTROLLER_TYPE:-mpc}" \
    --mount "type=bind,src=$XAUTHORITY,dst=/tmp/carla-xauthority,readonly" \
    --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
    --mount "type=bind,src=$demo_results,dst=/demo-results" \
    --mount "type=bind,src=$demo_dir/carla_local_path.rviz,dst=/workspace/install/carla_ad_demo/share/carla_ad_demo/config/carla_ad_demo_ros2.rviz,readonly" \
    --mount "type=bind,src=$demo_dir/fixed-objects-gui.json,dst=/workspace/install/carla_spawn_objects/share/carla_spawn_objects/config/objects.json,readonly" \
    --mount "type=bind,src=$dev_dir,dst=/learning,readonly" \
    codex-carla-foxy:gui bash -lc \
    'set -e; source /opt/ros/foxy/setup.bash; source /workspace/install/setup.bash; source /learning/install/local_setup.bash; test "$(ros2 pkg prefix my_pnc)" = /learning/install/my_pnc; export PYTHONPATH=/opt/carla-source/PythonAPI/carla:$PYTHONPATH; ros2 launch carla_ad_demo carla_ad_demo.launch.py host:=127.0.0.1' \
    > "$demo_results/demo.log" 2>&1
