# 规控开发工作区

本目录挂载至开发容器 `carla-controller-dev` 的 `/learning` 路径，用于宿主机代码编辑与容器内增量编译。

- `src/my_planning_and_control/`：核心 C++17 / ROS2 Foxy 规控功能包源码（`general_modules`、`planner_modules`、`controller_modules`、`main_function`）。
- `start-dev.sh`：启动常驻开发容器。
- `build-from-host.sh`：从宿主机调用容器内 `colcon build` 完成增量编译。

日常使用方式请参考根目录 [`README.md`](../../README.md)。
