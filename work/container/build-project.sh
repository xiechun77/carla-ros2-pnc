#!/bin/bash
set -eo pipefail
source /opt/ros/foxy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
mkdir -p /workspace/src
cp -a /input/plus_version/. /workspace/src/
cd /workspace
colcon list
colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
