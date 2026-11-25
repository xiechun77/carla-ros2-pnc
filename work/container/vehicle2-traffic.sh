#!/bin/bash
set -eo pipefail
scenario_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-status}"
scenario_name=carla-vehicle2-traffic
case "$action" in
  stop)
    if docker container inspect "$scenario_name" >/dev/null 2>&1; then
      test "$(docker inspect "$scenario_name" --format '{{index .Config.Labels "codex.scenario"}}')" = vehicle2-traffic
      docker stop -t 15 "$scenario_name"
    else
      echo '背景车场景脚本未运行。'
    fi
    exit 0;;
  status|run) ;;
  *) echo '用法: bash vehicle2-traffic.sh status|run|stop'; exit 2;;
esac
mapfile -t demo_ids < <(docker ps --filter label=codex.carla.demo=1 --format '{{.ID}}')
if [[ ${#demo_ids[@]} -ne 1 ]]; then
  echo '需要恰好一个运行中的演示。请先启动 run-controller-experiment.sh，等待车辆出现。' >&2
  exit 1
fi
if [[ "$action" == run ]] && docker container inspect "$scenario_name" >/dev/null 2>&1; then
  echo 'vehicle_2 场景已存在；请先运行 bash vehicle2-traffic.sh stop。' >&2
  exit 1
fi
name_args=()
if [[ "$action" == run ]]; then name_args=(--name "$scenario_name"); fi
exec docker run --rm --init "${name_args[@]}" --label codex.scenario=vehicle2-traffic \
  --network "container:${demo_ids[0]}" --user "$(id -u):$(id -g)" \
  --workdir /tmp -e HOME=/tmp -e PYTHONPATH=/opt/carla-source/PythonAPI/carla \
  --mount "type=bind,src=$scenario_dir/scenario-vehicle2.py,dst=/scenario.py,readonly" \
  codex-carla-foxy:gui python3 -u /scenario.py "$action"
