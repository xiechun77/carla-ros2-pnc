#!/bin/bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
case "${1:-help}" in
  dev) exec bash "$project_dir/work/controller-dev/start-dev.sh";;
  build) exec bash "$project_dir/work/controller-dev/build-from-host.sh";;
  run) exec bash "$project_dir/work/container/run-controller-experiment.sh";;
  eval) shift; exec bash "$project_dir/work/container/run-eval.sh" "$@";;
  compare) shift; exec python3 "$project_dir/experiments/eval/compare.py" "$@";;
  follow) shift; exec bash "$project_dir/work/container/run-following-check.sh" "$@";;
  original) exec bash "$project_dir/work/container/run-demo.sh";;
  pause|play|step) exec bash "$project_dir/work/container/demo-control.sh" "$1";;
  stop) bash "$project_dir/work/container/crossing-traffic.sh" stop; bash "$project_dir/work/container/vehicle2-traffic.sh" stop; exec bash "$project_dir/work/container/stop-demo.sh";;
  ros) exec bash "$project_dir/work/container/enter-ros.sh";;
  traffic) exec bash "$project_dir/work/container/vehicle2-traffic.sh" "${2:-status}";;
  crossing) shift; exec bash "$project_dir/work/container/crossing-traffic.sh" "$@";;
  *) echo '用法: bash project.sh dev|build|run|eval [--planner em|lattice] [--controller mpc|lqr|stanley] [--scenario cruise] [--duration 30] [--headless|--gui]|compare <run1> <run2>...|follow [--gui|--headless]|original|pause|play|step|stop|ros|traffic [status|run|stop]|crossing [yield|pass|status|stop]' ;;
esac
