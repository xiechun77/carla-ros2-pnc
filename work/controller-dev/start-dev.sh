#!/bin/bash
set -eo pipefail
dev_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if docker container inspect carla-controller-dev >/dev/null 2>&1; then
  docker start carla-controller-dev >/dev/null
else
  docker run -d --name carla-controller-dev --init --network none \
    --label codex.task=carla-controller-learning \
    --user "$(id -u):$(id -g)" -e HOME=/learning/home \
    --mount "type=bind,src=$dev_dir,dst=/learning" \
    --workdir /learning codex-carla-foxy:gui sleep infinity
fi
echo 'VS Code: Attach to Running Container -> carla-controller-dev -> /learning'
