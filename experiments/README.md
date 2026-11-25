# 闭环评测与场景实验

本目录包含统一的规控闭环评测工具链、固化基准对比报告以及专项场景诊断配置：

- `eval/recorder.py`：ROS2 闭环指标实时采集节点，自动统计横向误差、航向误差、速度误差、纵向急动度、转向速率、最小障碍物距离、碰撞次数与规划/控制耗时（Mean / P95）。
- `eval/compare.py`：多组实验数据对齐汇总工具，自动生成 Markdown 定量指标表与零外部依赖的 SVG 多子图对比曲线。
- `benchmarks/`：`2 规划器 (EM / Lattice) × 3 控制器 (Stanley / LQR / MPC)` 全路线闭环基准数据与对比曲线图。
- `following/`：Town01 直道同车道低速前车跟驰场景配置（通过 `bash project.sh follow` 运行）。
