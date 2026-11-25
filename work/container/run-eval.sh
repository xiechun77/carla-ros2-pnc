#!/bin/bash
set -eo pipefail

container_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd -- "$container_dir/../.." && pwd)"
dev_dir="$root_dir/work/controller-dev"

if [[ -n "$(docker ps -q --filter label=codex.carla.demo=1)" ]]; then
    echo "请先结束现有演示，再启动评测。" >&2
    exit 1
fi

planner="em"
controller="mpc"
scenario="cruise"
duration="30"
spawn_point="127.4,-195.4,2,0,0,180"
mode="headless"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --planner) planner="$2"; shift 2 ;;
        --controller) controller="$2"; shift 2 ;;
        --scenario) scenario="$2"; shift 2 ;;
        --duration) duration="$2"; shift 2 ;;
        --spawn-point) spawn_point="$2"; shift 2 ;;
        --gui) mode="gui"; shift ;;
        --headless) mode="headless"; shift ;;
        *)
            echo "用法: run-eval.sh [--planner em|lattice] [--controller mpc|lqr|stanley] [--scenario cruise] [--duration 30] [--spawn-point X,Y,Z,R,P,Y] [--headless|--gui]" >&2
            exit 2
            ;;
    esac
done

test -f "$dev_dir/install/local_setup.bash"

run_id="$(date +%Y%m%d-%H%M%S)_${scenario}_${planner}_${controller}"
server_name="carla-eval-server-$$"
runner_name="carla-eval-ros-$$"
results_dir="$root_dir/experiments/eval/runs/$run_id"
mkdir -p "$results_dir"

cleanup() {
    if docker inspect "$runner_name" >/dev/null 2>&1; then
        docker logs "$runner_name" > "$results_dir/ros-docker.log" 2>&1 || true
    fi
    docker stop -t 15 "$runner_name" >/dev/null 2>&1 || true
    if docker inspect "$server_name" >/dev/null 2>&1; then
        docker logs "$server_name" > "$results_dir/server.log" 2>&1 || true
    fi
    docker stop -t 10 "$server_name" >/dev/null 2>&1 || true
}
trap cleanup EXIT

ui_args=()
if [[ "$mode" == "gui" ]]; then
    : "${DISPLAY:?请从当前 Ubuntu 图形桌面终端运行}"
    : "${XAUTHORITY:?缺少 XAUTHORITY}"
    ui_args=(
        --ipc=host --gpus all --runtime=nvidia
        -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display
        -e DISPLAY -e XAUTHORITY=/tmp/carla-xauthority
        -e QT_X11_NO_MITSHM=1 -e SDL_VIDEODRIVER=x11 -e SDL_AUDIODRIVER=dummy
        --mount "type=bind,src=$XAUTHORITY,dst=/tmp/carla-xauthority,readonly"
        --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly
    )
else
    ui_args=(
        -e QT_QPA_PLATFORM=offscreen -e SDL_AUDIODRIVER=dummy -e SDL_VIDEODRIVER=dummy
    )
fi

printf '开始评测: planner=%s controller=%s scenario=%s duration=%ss mode=%s\n' \
    "$planner" "$controller" "$scenario" "$duration" "$mode"
printf '结果目录: %s\n' "$results_dir"

docker run -d --rm --init --name "$server_name" --label codex.task=carla-reproduction \
    --gpus all --runtime=nvidia --shm-size=2g -e NVIDIA_DRIVER_CAPABILITIES=all \
    carlasim/carla:0.9.13 bash ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -nosound >/dev/null

docker run --rm --network "container:$server_name" \
    --mount "type=bind,src=$container_dir/check-carla-server.py,dst=/check.py,readonly" \
    codex-carla-foxy:gui python3 /check.py > "$results_dir/readiness.log" 2>&1

docker run --rm --init --name "$runner_name" \
    --label codex.carla.demo=1 --label codex.task=carla-reproduction \
    --network "container:$server_name" \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e PNC_PLANNER_TYPE="$planner" \
    -e PNC_CONTROLLER_TYPE="$controller" \
    -e PNC_DEBUG_DIR=/results \
    -e PYTHONPATH=/opt/carla-source/PythonAPI/carla \
    "${ui_args[@]}" \
    --mount "type=bind,src=$dev_dir,dst=/learning,readonly" \
    --mount "type=bind,src=$results_dir,dst=/results" \
    --mount "type=bind,src=$root_dir/experiments/eval/recorder.py,dst=/eval-recorder.py,readonly" \
    --mount "type=bind,src=$container_dir/carla_local_path.rviz,dst=/workspace/install/carla_ad_demo/share/carla_ad_demo/config/carla_ad_demo_ros2.rviz,readonly" \
    --mount "type=bind,src=$container_dir/fixed-objects-gui.json,dst=/workspace/install/carla_spawn_objects/share/carla_spawn_objects/config/objects.json,readonly" \
    codex-carla-foxy:gui bash -lc "
        set -e
        source /opt/ros/foxy/setup.bash
        source /workspace/install/setup.bash
        source /learning/install/local_setup.bash
        test \"\$(ros2 pkg prefix my_pnc)\" = /learning/install/my_pnc
        PIDS=()
        stop_children() {
            for p in \"\${PIDS[@]}\"; do
                kill \"\$p\" 2>/dev/null || true
            done
        }
        trap stop_children EXIT
        ros2 launch carla_ros_bridge carla_ros_bridge.launch.py host:=127.0.0.1 town:=Town01 fixed_delta_seconds:=0.05 > /results/bridge.log 2>&1 &
        PIDS+=(\$!)
        sleep 4
        ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py host:=127.0.0.1 > /results/waypoints.log 2>&1 &
        PIDS+=(\$!)
        ros2 run carla_spawn_objects carla_spawn_objects --ros-args \
            -p objects_definition_file:=/workspace/install/carla_spawn_objects/share/carla_spawn_objects/config/objects.json \
            -p 'spawn_point_ego_vehicle:=$spawn_point' > /results/spawn.log 2>&1 &
        PIDS+=(\$!)
        ros2 launch my_pnc my_pnc_launch.py planner_type:='$planner' controller_type:='$controller' > /results/pnc.log 2>&1 &
        PIDS+=(\$!)
        if [[ '$mode' == 'gui' ]]; then
            ros2 run rviz2 rviz2 -d /workspace/install/carla_ad_demo/share/carla_ad_demo/config/carla_ad_demo_ros2.rviz > /results/rviz.log 2>&1 &
            PIDS+=(\$!)
        fi
        python3 -u /eval-recorder.py \
            --output-dir /results \
            --scenario '$scenario' \
            --planner '$planner' \
            --controller '$controller' \
            --duration '$duration'
        stop_children
    "

printf '评测完成！查看结果:\n  %s/summary.json\n  %s/samples.csv\n' "$results_dir" "$results_dir"
