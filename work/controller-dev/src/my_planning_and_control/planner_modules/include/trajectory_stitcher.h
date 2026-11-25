#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

#include "common.h"

/**
 * @brief 规划起点计算与轨迹拼接模块（参考 Apollo TrajectoryStitcher）
 *
 * 供 EMPlanner 与 LatticePlanner 共用，确保不同规划器在规划起点外推、
 * 偏差重规划判定以及历史轨迹拼接逻辑上完全一致。
 */
class TrajectoryStitcher
{
public:
    TrajectoryStitcher() = default;

    // 根据当前自车状态与上一周期轨迹，计算本周期规划起点并准备待拼接的历史段
    TrajectoryPoint compute_planning_start_point(
        double current_time,
        double planning_cycle_time,
        const std::shared_ptr<VehicleState>& ego_state,
        double max_lateral_error = 0.5,
        double max_longitudinal_error = 1.5,
        size_t preserved_history_points = 20);

    // 将历史拼接段与本周期新规划轨迹合并为最终轨迹，并更新上一周期缓存
    void stitch_and_update(
        const std::vector<TrajectoryPoint>& new_trajectory,
        std::vector<TrajectoryPoint>& final_trajectory);

    // 清空历史轨迹缓存（切换路线或重启时调用）
    void reset();

    bool replan_triggered() const { return _replan_triggered; }
    double last_lateral_error() const { return _last_lateral_error; }
    double last_longitudinal_error() const { return _last_longitudinal_error; }

private:
    static size_t find_nearest_time_index(
        const std::deque<TrajectoryPoint>& trajectory,
        double target_time);

    static TrajectoryPoint compute_replan_start_point(
        double current_time,
        double planning_cycle_time,
        const std::shared_ptr<VehicleState>& ego_state);

    std::deque<TrajectoryPoint> _previous_trajectory;
    std::deque<TrajectoryPoint> _stitch_trajectory;
    bool _replan_triggered = true;
    double _last_lateral_error = 0.0;
    double _last_longitudinal_error = 0.0;
};
