# 同车道低速前车跟驰测试场景

本目录提供独立的直道跟驰测试场景配置，用于验证规划器与纵向控制器在同车道低速动态前车工况下的速度规划与安全车距保持表现。

## 运行方式

```bash
# 图形界面模式（含 RViz2 与车辆跟随视角）
bash project.sh follow

# 无界面批处理模式
bash project.sh follow --headless
```

## 场景配置说明

- `objects.json`：仅生成自车与同车道前车 `vehicle_2`（Town01 直道，初始纵向中心间距约 30 m，前车目标巡航速度约 10 km/h）。
- `scenario.json`：定义前车暖机时长、观察窗口时长与安全停车退出阈值。
- 每次运行结果自动保存在本地 `results/<timestamp>/` 目录下（包含 `samples.csv`、`summary.json` 与节点日志）。
