#!/bin/bash
set -eo pipefail
mapfile -t learning_demo_ids < <(docker ps --filter label=codex.carla.demo=1 --format '{{.ID}}')
if [[ ${#learning_demo_ids[@]} -ne 1 ]]; then
    printf '%s\n' '需要恰好一个由run-demo.sh启动的演示；没有停止其他容器。'
    exit 1
fi
learning_demo_id="${learning_demo_ids[0]}"
learning_network_mode=$(docker inspect "$learning_demo_id" --format '{{.HostConfig.NetworkMode}}')
learning_server_id="${learning_network_mode#container:}"
if [[ "$learning_network_mode" != container:* ]] || [[ $(docker inspect "$learning_server_id" --format '{{index .Config.Labels "codex.task"}}') != carla-reproduction ]]; then
    printf '%s\n' '无法确认配套CARLA服务，未停止容器。'
    exit 1
fi
docker stop -t 12 "$learning_demo_id"
if docker inspect "$learning_server_id" >/dev/null 2>&1; then
    docker stop -t 10 "$learning_server_id" || true
fi
