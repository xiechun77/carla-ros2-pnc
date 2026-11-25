#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "common.h"
#include "lattice_components.h"
#include "lattice_curve1d.h"
#include "planner_base.h"
#include "planning_debug.h"
#include "trajectory_stitcher.h"

/**
 * @brief Apollo 9.0 风格模块化 Lattice 规划器（零 CyberRT 依赖）
 *
 * 核心流水线（严格对标 Apollo LatticePlanner::PlanOnReferenceLine）：
 *  1. TrajectoryStitcher 计算规划起点与历史拼接段
 *  2. 参考线弧长参数化与笛卡尔 -> Frenet 三阶初态转换 (init_s, init_d)
 *  3. PathTimeGraph 障碍物分类与动态 ST 阻塞区间投影
 *  4. Trajectory1dGenerator 生成纵向（巡航/停车/跟车/超车）与横向（车道保持/借道绕障）1D 轨迹束
 *  5. TrajectoryEvaluator 最小堆惰性排序 + TrajectoryCombiner 合成 2D 轨迹
 *  6. ConstraintChecker 六维动力学硬约束 + CollisionChecker 2D 有向包围盒（SAT）碰撞检测
 *  7. BackupTrajectoryGenerator 极端无解工况下的安全减速兜底轨迹
 */
class LatticePlanner : public PlannerBase
{
public:
    LatticePlanner();
    ~LatticePlanner() override = default;

    std::string name() const override { return "lattice"; }

    void plan(
        double current_time,
        const std::vector<PathPoint>& reference_line,
        const std::shared_ptr<VehicleState>& ego_state,
        const std::vector<derived_object_msgs::msg::Object>& obstacles,
        std::vector<TrajectoryPoint>& final_trajectory) override;

    const PlanningDebugData& debug_data() const override { return _debug_data; }

    void reset() override
    {
        _stitcher.reset();
    }

private:
    void ComputeInitFrenetState(
        const std::vector<PathPoint>& reference_line,
        const std::vector<double>& ref_s,
        const TrajectoryPoint& start_point,
        std::array<double, 3>* init_s,
        std::array<double, 3>* init_d) const;

    std::vector<TrajectoryPoint> CombineTrajectory(
        const std::vector<PathPoint>& reference_line,
        const std::vector<double>& ref_s,
        const lattice::Curve1d& lon_traj,
        const lattice::Curve1d& lat_traj,
        double start_timestamp,
        double dt_step,
        double left_bound_d,
        double right_bound_d,
        bool* out_within_road_bound) const;

    bool IsTrajectoryDynamicallyValid(
        const std::vector<TrajectoryPoint>& trajectory) const;

    bool IsTrajectoryCollisionFree(
        const std::vector<TrajectoryPoint>& trajectory,
        double start_timestamp,
        const lattice::PathTimeGraph& pt_graph) const;

    std::vector<TrajectoryPoint> GenerateBackupTrajectory(
        const std::vector<PathPoint>& reference_line,
        const std::vector<double>& ref_s,
        const std::array<double, 3>& init_s,
        const std::array<double, 3>& init_d,
        const lattice::PathTimeGraph& pt_graph,
        double start_timestamp) const;

    void PopulateDebugData(
        double current_time,
        double compute_ms,
        const std::vector<PathPoint>& reference_line,
        const std::vector<double>& ref_s,
        const std::shared_ptr<VehicleState>& ego_state,
        const std::array<double, 3>& init_s,
        const std::array<double, 3>& init_d,
        const lattice::PathTimeGraph& pt_graph,
        const std::shared_ptr<lattice::Curve1d>& best_lon,
        const std::shared_ptr<lattice::Curve1d>& best_lat,
        bool used_backup);

    lattice::LatticeConfig _cfg;
    TrajectoryStitcher _stitcher;
    PlanningDebugData _debug_data;
};
