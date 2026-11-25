#!/bin/bash
set -eo pipefail
scenario_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-yield}"
shift || true
scenario_name=carla-crossing-pedestrian
case "$action" in
  stop)
    if docker container inspect "$scenario_name" >/dev/null 2>&1; then
      test "$(docker inspect "$scenario_name" --format '{{index .Config.Labels "codex.scenario"}}')" = crossing-pedestrian
      docker stop -t 15 "$scenario_name"
    else
      echo '横穿行人场景脚本未运行。'
    fi
    exit 0;;
  status|yield|pass) ;;
  *) echo '用法: bash crossing-traffic.sh yield|pass|status|stop [额外参数如 --trigger-dist 24 --speed 1.8]'; exit 2;;
esac
mapfile -t demo_ids < <(docker ps --filter label=codex.carla.demo=1 --format '{{.ID}}')
if [[ ${#demo_ids[@]} -ne 1 ]]; then
  echo '需要恰好一个运行中的演示。请先启动 bash project.sh run，等待车辆出现。' >&2
  exit 1
fi
if [[ "$action" != status ]] && docker container inspect "$scenario_name" >/dev/null 2>&1; then
  echo '横穿行人场景已在运行；请先执行 bash project.sh crossing stop。' >&2
  exit 1
fi
name_args=()
if [[ "$action" != status ]]; then name_args=(--name "$scenario_name"); fi
exec docker run --rm --init "${name_args[@]}" --label codex.scenario=crossing-pedestrian \
  --network "container:${demo_ids[0]}" --user "$(id -u):$(id -g)" \
  --workdir /tmp -e HOME=/tmp -e PYTHONPATH=/opt/carla-source/PythonAPI/carla \
  --mount "type=bind,src=$scenario_dir/scenario-crossing.py,dst=/scenario.py,readonly" \
  codex-carla-foxy:gui python3 -u /scenario.py "$action" "$@"
