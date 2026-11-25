# 容器环境与仿真编排脚本

本目录包含 ROS2 Foxy 规控运行镜像的构建配置、RViz2 可视化配置以及闭环仿真与自动化评测启动脚本。

## 主要文件说明

- `Dockerfile` / `Dockerfile.gui`：Ubuntu 20.04 + ROS2 Foxy + OSQP 0.6.3 + OSQP-Eigen 0.8.0 + Matplot++ 1.2.1 + CARLA 0.9.13 Agents 依赖与图形环境构建配置。
- `run-controller-experiment.sh`：启动带 RViz2 与车辆视角的交互式闭环仿真，支持通过 `PNC_PLANNER_TYPE` 与 `PNC_CONTROLLER_TYPE` 指定规划器与控制器。
- `run-eval.sh`：确定性闭环定量评测编排脚本（支持 `--headless` 与 `--gui`）。
- `fixed-objects-gui.json`：Town01 传感器配置与静态障碍车场景定义。
- `carla_local_path.rviz` / `carla_ad_demo_foxy_compatible.rviz`：RViz2 视图配置文件。
- `vehicle2-traffic.sh` / `crossing-traffic.sh`：动态前车与路口横穿车辆场景控制脚本。
- `demo-control.sh` / `stop-demo.sh`：仿真时钟暂停/单步控制与容器清理脚本。
