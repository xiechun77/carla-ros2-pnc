#pragma once

#include <array>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common.h"
#include "derived_object_msgs/msg/object.hpp"
#include "lattice_curve1d.h"

namespace lattice {

struct LatticeConfig
{
    double trajectory_time_length = 5.0;       // 规划时域 T (s)
    double trajectory_time_resolution = 0.1;   // 评估采样时间步 (s)
    double output_time_resolution = 0.02;      // 输出给控制器的加密步长 (s)
    double space_resolution = 1.0;             // 横向评估空间步长 (m)

    // 车辆与道路几何参数
    double ego_length = 4.7;
    double ego_width = 2.0;
    double lane_width = 4.0;

    // 纵向与横向动力学硬约束（ConstraintChecker）
    double max_lon_vel = 15.0;                 // m/s
    double min_lon_vel = -0.05;                // 禁止倒车
    double max_lon_acc = 4.0;                  // m/s^2
    double min_lon_acc = -5.5;                 // m/s^2
    double max_lon_jerk = 8.0;                 // m/s^3
    double max_lat_acc = 5.8;                  // m/s^2（留足中速近距离借道变道的瞬态向心加速度余量）
    double max_curvature = 0.60;               // 1/m（匹配 CARLA Tesla Model 3 轴距 2.852m 与最大前轮转角，覆盖 Town01 90° 急弯）

    // 碰撞安全缓冲（对标 Apollo lat_collision_buffer=0.1~0.25m）
    double static_lon_buffer = 0.70;           // 静态障碍物纵向膨胀缓冲 (m)
    double static_lat_buffer = 0.25;           // 静态障碍物横向安全间距 (m)
    double dynamic_lon_buffer = 2.2;           // 动态障碍物纵向缓冲 (m)
    double dynamic_lat_buffer = 0.45;          // 动态障碍物横向缓冲 (m)
    double lon_yield_buffer = 4.5;             // ST 跟车让行采样缓冲 (m)
    double lon_overtake_buffer = 5.5;          // ST 超车采样缓冲 (m)

    // Apollo TrajectoryEvaluator 代价权重（严格对齐 Apollo 官方比例：里程进展权重 > 速度跟踪权重）
    double weight_lon_objective = 20.0;
    double weight_lon_jerk = 1.0;
    double weight_lon_collision = 5.0;
    double weight_centripetal_acc = 1.5;
    double weight_lat_offset = 3.0;
    double weight_lat_comfort = 1.5;
    double weight_target_speed = 2.0;
    double weight_dist_travelled = 10.0;
    double lat_offset_bound = 3.0;
};

struct ObstacleSLBox
{
    uint32_t id = 0;
    bool is_static = true;
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double speed = 0.0;
    double length = 4.6;
    double width = 2.0;

    // 参考线 Frenet 投影
    double s = 0.0;
    double l = 0.0;
    double s_dot = 0.0;
    double l_dot = 0.0;
};

struct STObstacleInterval
{
    uint32_t id = 0;
    double t_in = 0.0;
    double t_out = 0.0;
    double s_in = 0.0;
    double s_out = 0.0;
    double v_lon = 0.0;
};

struct PlanningTarget
{
    double cruise_speed = 12.0;
    bool has_stop_point = false;
    double stop_s = 0.0;
};

/**
 * @brief 动态/静态障碍物感知查询与 ST 图构建（对标 Apollo PathTimeGraph + PredictionQuerier）
 */
class PathTimeGraph
{
public:
    PathTimeGraph(
        const std::vector<derived_object_msgs::msg::Object>& raw_obstacles,
        const std::vector<PathPoint>& reference_line,
        const std::vector<double>& ref_s,
        const std::shared_ptr<VehicleState>& ego_state,
        const LatticeConfig& cfg);

    const std::vector<ObstacleSLBox>& obstacles() const { return _obstacles; }
    const std::vector<ObstacleSLBox>& static_obstacles() const { return _static_obstacles; }
    const std::vector<ObstacleSLBox>& dynamic_obstacles() const { return _dynamic_obstacles; }
    const std::vector<STObstacleInterval>& st_intervals() const { return _st_intervals; }

    // 获取每个离散时刻 t_i 上的纵向阻塞区间 [s_lower, s_upper]
    std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
        double t_start, double t_end, double dt) const;

    // 导出为 PlanningDebugData 兼容的 st_graph 字典列表
    std::vector<std::unordered_map<std::string, double>> ExportDebugSTGraph() const;

    // 判断对向（左侧借道）车道是否畅通，供横向采样器决定是否启用借道绕障采样
    bool IsLeftBorrowLaneClear() const { return _left_lane_clear; }
    bool HasBlockingInLaneObstacle() const { return _has_blocking_in_lane_obs; }

private:
    std::vector<ObstacleSLBox> _obstacles;
    std::vector<ObstacleSLBox> _static_obstacles;
    std::vector<ObstacleSLBox> _dynamic_obstacles;
    std::vector<STObstacleInterval> _st_intervals;
    bool _left_lane_clear = true;
    bool _has_blocking_in_lane_obs = false;
};

/**
 * @brief 一维横纵向轨迹束生成器（对标 Apollo EndConditionSampler + Trajectory1dGenerator）
 */
class Trajectory1dGenerator
{
public:
    Trajectory1dGenerator(
        const std::array<double, 3>& init_s,
        const std::array<double, 3>& init_d,
        const PathTimeGraph& pt_graph,
        const LatticeConfig& cfg);

    void GenerateTrajectoryBundles(
        const PlanningTarget& target,
        std::vector<std::shared_ptr<Curve1d>>* lon_bundle,
        std::vector<std::shared_ptr<Curve1d>>* lat_bundle) const;

private:
    void GenerateLongitudinalBundle(
        const PlanningTarget& target,
        std::vector<std::shared_ptr<Curve1d>>* lon_bundle) const;

    void GenerateLateralBundle(
        std::vector<std::shared_ptr<Curve1d>>* lat_bundle) const;

    std::array<double, 3> _init_s;
    std::array<double, 3> _init_d;
    const PathTimeGraph& _pt_graph;
    const LatticeConfig& _cfg;
};

/**
 * @brief 基于最小堆优先队列的轨迹对代价评估器（对标 Apollo TrajectoryEvaluator）
 */
class TrajectoryEvaluator
{
public:
    using PtrCurve1d = std::shared_ptr<Curve1d>;
    using Pair = std::pair<PtrCurve1d, PtrCurve1d>;

    TrajectoryEvaluator(
        const std::array<double, 3>& init_s,
        const PlanningTarget& target,
        const std::vector<PtrCurve1d>& lon_trajectories,
        const std::vector<PtrCurve1d>& lat_trajectories,
        const PathTimeGraph& pt_graph,
        const std::vector<PathPoint>& reference_line,
        const std::vector<double>& ref_s,
        const LatticeConfig& cfg);

    bool has_more_pairs() const { return !_cost_queue.empty(); }
    size_t num_of_pairs() const { return _cost_queue.size(); }
    double top_cost() const { return _cost_queue.top().second; }
    Pair next_top_pair();

private:
    using PairWithCost = std::pair<Pair, double>;
    struct CostComparator
    {
        bool operator()(const PairWithCost& a, const PairWithCost& b) const
        {
            return a.second > b.second;  // 最小堆：代价小者优先弹出
        }
    };

    bool IsValidLongitudinal1d(const Curve1d& lon_traj) const;
    double EvaluateCost(
        const PlanningTarget& target,
        const PtrCurve1d& lon_traj,
        const PtrCurve1d& lat_traj) const;

    double LonObjectiveCost(const PtrCurve1d& lon_traj, const PlanningTarget& target) const;
    double LonComfortCost(const PtrCurve1d& lon_traj) const;
    double LonCollisionCost(const PtrCurve1d& lon_traj) const;
    double CentripetalAccCost(const PtrCurve1d& lon_traj) const;
    double LatOffsetCost(const PtrCurve1d& lat_traj, double eval_s_len) const;
    double LatComfortCost(const PtrCurve1d& lon_traj, const PtrCurve1d& lat_traj) const;

    std::array<double, 3> _init_s;
    const PathTimeGraph& _pt_graph;
    const std::vector<PathPoint>& _reference_line;
    const std::vector<double>& _ref_s;
    const LatticeConfig& _cfg;
    std::vector<std::vector<std::pair<double, double>>> _blocking_intervals;
    std::vector<double> _ref_s_dots;
    std::priority_queue<PairWithCost, std::vector<PairWithCost>, CostComparator> _cost_queue;
};

}  // namespace lattice
