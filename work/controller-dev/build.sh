#!/bin/bash
set -eo pipefail
source /opt/ros/foxy/setup.bash
source /workspace/install/setup.bash
export CPLUS_INCLUDE_PATH=/usr/include/eigen3
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
cd /learning
colcon build --packages-select my_pnc --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source /learning/install/local_setup.bash
test "$(ros2 pkg prefix my_pnc)" = /learning/install/my_pnc
echo 'Verified: my_pnc resolves to /learning/install/my_pnc'
