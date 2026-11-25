#include "lateral_stanley_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

LateralStanleyController::LateralStanleyController() = default;

bool LateralStanleyController::compute_control_cmd(
    const std::vector<TrajectoryPoint>& trajectory,
    const std::shared_ptr<VehicleState>& ego_state,
    double /*cur_t*/,
    double /*dt*/,
    ControlCMD& cmd)
{
    if (trajectory.empty() || !ego_state) {
        return false;
    }

    // 1. 计算前轴中心在世界坐标系下的位置 (x_f, y_f)
    const double front_x = ego_state->x + _lf * std::cos(ego_state->heading);
    const double front_y = ego_state->y + _lf * std::sin(ego_state->heading);

    // 2. 在规划轨迹上搜索距离前轴中心最近的参考点
    size_t nearest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const double dx = front_x - trajectory[i].x;
        const double dy = front_y - trajectory[i].y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            nearest_idx = i;
        }
    }
    const auto& target_point = trajectory[nearest_idx];

    // 3. 切向连续投影与法向横向误差 ed（位于参考路径左侧为正）
    const Eigen::Vector2d tau(std::cos(target_point.heading), std::sin(target_point.heading));
    const Eigen::Vector2d nor(-std::sin(target_point.heading), std::cos(target_point.heading));
    const Eigen::Vector2d vec_d(front_x - target_point.x, front_y - target_point.y);
    const double es = tau.transpose() * vec_d;
    const double ed = nor.transpose() * vec_d;

    // 利用局部曲率对参考航向角做连续一阶插值修正，消除离散点切换引起的阶跃毛刺
    const double kappa = std::isfinite(target_point.kappa) ? target_point.kappa : 0.0;
    const double ref_heading = target_point.heading + kappa * es;
    const double ephi = normalize_angle(ego_state->heading - ref_heading);

    // 4. Stanley 控制律：航向消除项 + 横向误差反正切项 + 阿克曼曲率前馈
    const double vx = std::max(0.0, ego_state->v);
    const double cross_track_term = std::atan2(_k_y * ed, _k_soft + vx);
    const double curvature_ff = _k_ff * std::atan(_wheelbase * kappa);

    // 前轮转角 u（左转为正，与 LateralLQRController 符号约定一致）
    double u = -ephi - cross_track_term + curvature_ff;
    u = std::min(std::max(u, -_max_steer_rad), _max_steer_rad);

    // CARLA 方向盘控制指令（右转为正，范围 [-1, 1]）及一阶平滑滤波
    double raw_steer = std::min(std::max(-u, -1.0), 1.0);
    if (_has_prev_steer) {
        raw_steer = _steer_filter_alpha * raw_steer + (1.0 - _steer_filter_alpha) * _prev_steer;
    }
    _prev_steer = raw_steer;
    _has_prev_steer = true;

    cmd.steer = std::min(std::max(raw_steer, -1.0), 1.0);
    return true;
}
