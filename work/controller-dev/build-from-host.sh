#!/bin/bash
set -eo pipefail
dev_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bash "$dev_dir/start-dev.sh"
docker exec carla-controller-dev bash /learning/build.sh
