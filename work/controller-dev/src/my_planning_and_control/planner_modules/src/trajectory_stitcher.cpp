#include "trajectory_stitcher.h"

#include <cmath>
#include "Eigen/Dense"

size_t TrajectoryStitcher::find_nearest_time_index(
    const std::deque<TrajectoryPoint>& trajectory,
    double target_time)
{
    if (trajectory.empty() || target_time <= trajectory.front().time_stamped)
    {
        return 0;
    }
    if (target_time >= trajectory.back().time_stamped)
    {
        return trajectory.size() - 1;
    }
    for (size_t i = 0; i + 1 < trajectory.size(); ++i)
    {
        if (target_time > trajectory[i].time_stamped &&
            target_time <= trajectory[i + 1].time_stamped)
        {
            const double dt_prev = target_time - trajectory[i].time_stamped;
            const double dt_next = trajectory[i + 1].time_stamped - target_time;
            return (dt_prev <= dt_next) ? i : (i + 1);
        }
    }
    return trajectory.size() - 1;
}

TrajectoryPoint TrajectoryStitcher::compute_replan_start_point(
    double current_time,
    double planning_cycle_time,
    const std::shared_ptr<VehicleState>& ego_state)
{
    TrajectoryPoint start_point;
    start_point.ax = ego_state->ax;
    start_point.ay = ego_state->ay;
    const double ego_vx = ego_state->v * std::cos(ego_state->heading);
    const double ego_vy = ego_state->v * std::sin(ego_state->heading);
    const double next_vx = ego_vx + ego_state->ax * planning_cycle_time;
    const double next_vy = ego_vy + ego_state->ay * planning_cycle_time;
    start_point.v = std::hypot(next_vx, next_vy);
    start_point.heading = (start_point.v > 1e-3)
        ? std::atan2(next_vy, next_vx)
        : ego_state->heading;
    start_point.x = ego_state->x + ego_vx * planning_cycle_time +
                    0.5 * ego_state->ax * planning_cycle_time * planning_cycle_time;
    start_point.y = ego_state->y + ego_vy * planning_cycle_time +
                    0.5 * ego_state->ay * planning_cycle_time * planning_cycle_time;
    start_point.kappa = 0.0;
    start_point.time_stamped = current_time + planning_cycle_time;
    return start_point;
}

TrajectoryPoint TrajectoryStitcher::compute_planning_start_point(
    double current_time,
    double planning_cycle_time,
    const std::shared_ptr<VehicleState>& ego_state,
    double max_lateral_error,
    double max_longitudinal_error,
    size_t preserved_history_points)
{
    _stitch_trajectory.clear();
    _last_lateral_error = 0.0;
    _last_longitudinal_error = 0.0;

    // 首次规划或历史轨迹点不足：直接以当前位置作为零初速起点
    if (_previous_trajectory.size() < 2)
    {
        _replan_triggered = true;
        TrajectoryPoint start_point;
        start_point.x = ego_state->x;
        start_point.y = ego_state->y;
        start_point.v = 0.0;
        start_point.heading = ego_state->heading;
        start_point.ax = 0.0;
        start_point.ay = 0.0;
        start_point.kappa = 0.0;
        start_point.time_stamped = current_time + planning_cycle_time;
        return start_point;
    }

    // 历史轨迹已过期，直接按动力学外推重规划
    if (current_time > _previous_trajectory.back().time_stamped)
    {
        _replan_triggered = true;
        return compute_replan_start_point(current_time, planning_cycle_time, ego_state);
    }

    // 计算当前时刻自车与上一周期参考点之间的横向/纵向偏差
    const size_t current_time_index = find_nearest_time_index(_previous_trajectory, current_time);
    const auto& target_point = _previous_trajectory[current_time_index];
    const Eigen::Vector2d tau_target(std::cos(target_point.heading), std::sin(target_point.heading));
    const Eigen::Vector2d nor_target(-std::sin(target_point.heading), std::cos(target_point.heading));
    const Eigen::Vector2d host_to_target(target_point.x - ego_state->x, target_point.y - ego_state->y);

    _last_longitudinal_error = std::abs(host_to_target.dot(tau_target));
    _last_lateral_error = std::abs(host_to_target.dot(nor_target));

    if (_last_lateral_error < max_lateral_error &&
        _last_longitudinal_error < max_longitudinal_error)
    {
        _replan_triggered = false;
        const double start_time = current_time + planning_cycle_time;
        const size_t start_time_index = find_nearest_time_index(_previous_trajectory, start_time);

        TrajectoryPoint start_point = _previous_trajectory[start_time_index];
        start_point.time_stamped = start_time;

        // 截取 start_time_index 之前最多 preserved_history_points 个历史点用于轨迹拼接
        if (start_time_index > 0)
        {
            const size_t begin_idx = (start_time_index > preserved_history_points)
                ? (start_time_index - preserved_history_points)
                : 0;
            for (size_t i = begin_idx; i < start_time_index; ++i)
            {
                _stitch_trajectory.emplace_back(_previous_trajectory[i]);
            }
        }
        return start_point;
    }

    // 偏差过大，按动力学方程外推重规划
    _replan_triggered = true;
    return compute_replan_start_point(current_time, planning_cycle_time, ego_state);
}

void TrajectoryStitcher::stitch_and_update(
    const std::vector<TrajectoryPoint>& new_trajectory,
    std::vector<TrajectoryPoint>& final_trajectory)
{
    final_trajectory.clear();
    final_trajectory.reserve(_stitch_trajectory.size() + new_trajectory.size());
    for (const auto& pt : _stitch_trajectory)
    {
        final_trajectory.emplace_back(pt);
    }
    for (const auto& pt : new_trajectory)
    {
        final_trajectory.emplace_back(pt);
    }

    _previous_trajectory.clear();
    for (const auto& pt : final_trajectory)
    {
        _previous_trajectory.emplace_back(pt);
    }
}

void TrajectoryStitcher::reset()
{
    _previous_trajectory.clear();
    _stitch_trajectory.clear();
    _replan_triggered = true;
    _last_lateral_error = 0.0;
    _last_longitudinal_error = 0.0;
}
