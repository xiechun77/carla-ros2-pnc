#include "lattice_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {

auto LATTICE_LOG = rclcpp::get_logger("lattice_planner");

struct RefSample
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double kappa = 0.0;
    double s = 0.0;
};

// 按弧长 s 在离散参考线上做连续角度安全插值
RefSample SampleReferenceLineByS(
    const std::vector<PathPoint>& ref_line,
    const std::vector<double>& ref_s,
    double s)
{
    RefSample out;
    if (ref_line.empty() || ref_s.empty()) {
        return out;
    }
    if (s <= ref_s.front()) {
        const auto& p = ref_line.front();
        const double ds = s - ref_s.front();
        out.x = p.x + ds * std::cos(p.heading);
        out.y = p.y + ds * std::sin(p.heading);
        out.heading = normalize_angle(p.heading + p.kappa * ds);
        out.kappa = p.kappa;
        out.s = s;
        return out;
    }
    if (s >= ref_s.back()) {
        const auto& p = ref_line.back();
        const double ds = s - ref_s.back();
        out.x = p.x + ds * std::cos(p.heading);
        out.y = p.y + ds * std::sin(p.heading);
        out.heading = normalize_angle(p.heading + p.kappa * ds);
        out.kappa = p.kappa;
        out.s = s;
        return out;
    }

    auto it = std::upper_bound(ref_s.begin(), ref_s.end(), s);
    const size_t i1 = std::min(static_cast<size_t>(std::distance(ref_s.begin(), it)), ref_line.size() - 1);
    const size_t i0 = (i1 > 0) ? (i1 - 1) : 0;
    const double seg_len = ref_s[i1] - ref_s[i0];
    const double alpha = (seg_len > 1e-6) ? ((s - ref_s[i0]) / seg_len) : 0.0;

    const auto& p0 = ref_line[i0];
    const auto& p1 = ref_line[i1];
    out.x = p0.x + alpha * (p1.x - p0.x);
    out.y = p0.y + alpha * (p1.y - p0.y);
    out.heading = normalize_angle(p0.heading + alpha * normalize_angle(p1.heading - p0.heading));
    // 抑制离散全局航点有限差分引起的单点曲率尖峰（城市路口真实几何曲率 <= 0.32 m^-1）
    const double raw_kappa = p0.kappa + alpha * (p1.kappa - p0.kappa);
    out.kappa = std::min(std::max(raw_kappa, -0.32), 0.32);
    out.s = s;
    return out;
}

// 2D 有向包围盒（OBB）分离轴定理（SAT）精确碰撞检测
bool CheckOBBOverlap(
    double x1, double y1, double yaw1, double half_len1, double half_wid1,
    double x2, double y2, double yaw2, double half_len2, double half_wid2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    // 快速外接圆排除
    const double r_max = std::hypot(half_len1, half_wid1) + std::hypot(half_len2, half_wid2);
    if (dx * dx + dy * dy > r_max * r_max) {
        return false;
    }

    const double c1 = std::cos(yaw1), s1 = std::sin(yaw1);
    const double c2 = std::cos(yaw2), s2 = std::sin(yaw2);

    // 4 个单位分离轴：(c1,s1), (-s1,c1), (c2,s2), (-s2,c2)
    const std::array<std::pair<double, double>, 4> axes{{
        {c1, s1}, {-s1, c1}, {c2, s2}, {-s2, c2}
    }};

    for (const auto& ax : axes) {
        const double ux = ax.first;
        const double uy = ax.second;
        const double center_proj = std::abs(dx * ux + dy * uy);
        const double r1 = half_len1 * std::abs(c1 * ux + s1 * uy) +
                          half_wid1 * std::abs(-s1 * ux + c1 * uy);
        const double r2 = half_len2 * std::abs(c2 * ux + s2 * uy) +
                          half_wid2 * std::abs(-s2 * ux + c2 * uy);
        if (center_proj > r1 + r2) {
            return false;
        }
    }
    return true;
}

}  // namespace

LatticePlanner::LatticePlanner()
{
    _reference_speed = 12.0;
}

void LatticePlanner::ComputeInitFrenetState(
    const std::vector<PathPoint>& reference_line,
    const std::vector<double>& ref_s,
    const TrajectoryPoint& start_point,
    std::array<double, 3>* init_s,
    std::array<double, 3>* init_d) const
{
    size_t best_i = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < reference_line.size(); ++i) {
        const double dx = start_point.x - reference_line[i].x;
        const double dy = start_point.y - reference_line[i].y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_i = i;
        }
    }

    const auto& mp = reference_line[best_i];
    const double cos_m = std::cos(mp.heading);
    const double sin_m = std::sin(mp.heading);
    const double tau_proj = (start_point.x - mp.x) * cos_m + (start_point.y - mp.y) * sin_m;

    const double rs = ref_s[best_i] + tau_proj;
    const double rx = mp.x + tau_proj * cos_m;
    const double ry = mp.y + tau_proj * sin_m;
    const double rtheta = normalize_angle(mp.heading + mp.kappa * tau_proj);
    const double rkappa = mp.kappa;

    const double dx = start_point.x - rx;
    const double dy = start_point.y - ry;
    const double d = -dx * std::sin(rtheta) + dy * std::cos(rtheta);

    const double one_minus_kd = std::max(0.2, 1.0 - rkappa * d);
    const double delta_theta = normalize_angle(start_point.heading - rtheta);
    const double cos_delta = std::max(0.15, std::cos(delta_theta));
    const double tan_delta = std::sin(delta_theta) / cos_delta;

    const double d_prime = std::min(std::max(one_minus_kd * tan_delta, -0.8), 0.8);
    // 与 EMPlanner 一致，将二阶横向导数初值置零以避免离散参考线曲率差分噪声引起五次多项式外拱过冲
    const double d_pprime = 0.0;

    const double s_dot = std::max(0.0, start_point.v * cos_delta / one_minus_kd);
    const double a_lon_proj = start_point.ax * std::cos(start_point.heading) +
                              start_point.ay * std::sin(start_point.heading);
    const double a_tau = (std::abs(start_point.a_tau) > 1e-4) ? start_point.a_tau : a_lon_proj;
    const double s_ddot = std::min(
        std::max(a_tau * cos_delta / one_minus_kd, _cfg.min_lon_acc), _cfg.max_lon_acc);

    *init_s = {{rs, s_dot, s_ddot}};
    *init_d = {{d, d_prime, d_pprime}};
}

std::vector<TrajectoryPoint> LatticePlanner::CombineTrajectory(
    const std::vector<PathPoint>& reference_line,
    const std::vector<double>& ref_s,
    const lattice::Curve1d& lon_traj,
    const lattice::Curve1d& lat_traj,
    double start_timestamp,
    double dt_step,
    double left_bound_d,
    double right_bound_d,
    bool* out_within_road_bound) const
{
    std::vector<TrajectoryPoint> combined;
    *out_within_road_bound = true;

    const double s0 = lon_traj.Evaluate(0, 0.0);
    const double s_ref_max = ref_s.back();
    double last_s = s0;

    const size_t reserve_n = static_cast<size_t>(_cfg.trajectory_time_length / dt_step) + 2;
    combined.reserve(reserve_n);

    for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += dt_step) {
        double s = std::max(last_s, lon_traj.Evaluate(0, t));
        last_s = s;
        if (s > s_ref_max) {
            break;
        }

        const double s_dot = std::max(0.0, lon_traj.Evaluate(1, t));
        const double s_ddot = lon_traj.Evaluate(2, t);
        const double rel_s = std::max(0.0, s - s0);

        const double d = lat_traj.Evaluate(0, rel_s);
        // 前 15m 允许从偶发偏界初态平滑回正，15m 之后约束在物理可行驶路面 [-1.10m, left_bound_d] 内
        const double strict_right = (rel_s > 15.0) ? -1.10 : right_bound_d;
        const double strict_left = (rel_s > 15.0) ? std::min(left_bound_d, 3.45) : left_bound_d;
        if (d > strict_left || d < strict_right) {
            *out_within_road_bound = false;
            return combined;
        }
        const double d_prime = lat_traj.Evaluate(1, rel_s);
        const double d_pprime = lat_traj.Evaluate(2, rel_s);

        const RefSample ref = SampleReferenceLineByS(reference_line, ref_s, s);
        const double one_minus_kd = std::max(0.15, 1.0 - ref.kappa * d);
        const double delta_theta = std::atan2(d_prime, one_minus_kd);
        const double cos_delta = std::max(0.1, std::cos(delta_theta));
        const double tan_delta = std::sin(delta_theta) / cos_delta;

        TrajectoryPoint pt;
        pt.x = ref.x - d * std::sin(ref.heading);
        pt.y = ref.y + d * std::cos(ref.heading);
        pt.heading = normalize_angle(ref.heading + delta_theta);
        pt.kappa = (((d_pprime + ref.kappa * d_prime * tan_delta) * cos_delta * cos_delta) /
                        one_minus_kd +
                    ref.kappa) *
                   cos_delta / one_minus_kd;
        pt.v = s_dot * std::hypot(one_minus_kd, d_prime);
        pt.a_tau = s_ddot * one_minus_kd / cos_delta +
                   (s_dot * s_dot / cos_delta) *
                       (d_prime * (pt.kappa * one_minus_kd / cos_delta - ref.kappa) -
                        ref.kappa * d_prime);
        pt.ax = pt.a_tau * std::cos(pt.heading) - pt.v * pt.v * pt.kappa * std::sin(pt.heading);
        pt.ay = pt.a_tau * std::sin(pt.heading) + pt.v * pt.v * pt.kappa * std::cos(pt.heading);
        pt.time_stamped = start_timestamp + t;

        combined.emplace_back(pt);
    }
    return combined;
}

bool LatticePlanner::IsTrajectoryDynamicallyValid(
    const std::vector<TrajectoryPoint>& trajectory) const
{
    if (trajectory.size() < 4) {
        return false;
    }
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
            !std::isfinite(pt.v) || !std::isfinite(pt.kappa)) {
            return false;
        }
        if (pt.v < _cfg.min_lon_vel || pt.v > _cfg.max_lon_vel + 0.5) {
            return false;
        }
        if (pt.a_tau < _cfg.min_lon_acc - 0.5 || pt.a_tau > _cfg.max_lon_acc + 0.5) {
            return false;
        }
        if (std::abs(pt.kappa) > _cfg.max_curvature) {
            return false;
        }
        const double lat_acc = pt.v * pt.v * std::abs(pt.kappa);
        if (lat_acc > _cfg.max_lat_acc) {
            return false;
        }
    }
    return true;
}

bool LatticePlanner::IsTrajectoryCollisionFree(
    const std::vector<TrajectoryPoint>& trajectory,
    double start_timestamp,
    const lattice::PathTimeGraph& pt_graph) const
{
    const auto& obstacles = pt_graph.obstacles();
    if (obstacles.empty()) {
        return true;
    }

    const double ego_hl = 0.5 * _cfg.ego_length;
    const double ego_hw = 0.5 * _cfg.ego_width;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];
        const double rel_t = std::max(0.0, pt.time_stamped - start_timestamp);
        // 起点附近 (t < 0.5s) 渐进放大安全膨胀缓冲：避免自车已处于软缓冲层内时在 t=0 处将所有向外驶离的安全轨迹一刀切误杀
        const double buf_scale = std::min(1.0, std::max(0.15, rel_t / 0.5));

        for (const auto& obs : obstacles) {
            const double obs_x = obs.is_static ? obs.x : (obs.x + obs.vx * rel_t);
            const double obs_y = obs.is_static ? obs.y : (obs.y + obs.vy * rel_t);
            const double lon_buf = buf_scale * (obs.is_static ? _cfg.static_lon_buffer : _cfg.dynamic_lon_buffer);
            const double lat_buf = buf_scale * (obs.is_static ? _cfg.static_lat_buffer : _cfg.dynamic_lat_buffer);

            const double obs_hl = 0.5 * obs.length + lon_buf;
            const double obs_hw = 0.5 * obs.width + lat_buf;

            if (CheckOBBOverlap(pt.x, pt.y, pt.heading, ego_hl, ego_hw,
                                obs_x, obs_y, obs.heading, obs_hl, obs_hw)) {
                return false;
            }
        }
    }
    return true;
}

std::vector<TrajectoryPoint> LatticePlanner::GenerateBackupTrajectory(
    const std::vector<PathPoint>& reference_line,
    const std::vector<double>& ref_s,
    const std::array<double, 3>& init_s,
    const std::array<double, 3>& init_d,
    const lattice::PathTimeGraph& pt_graph,
    double start_timestamp) const
{
    // 安全兜底轨迹：
    // 1) 若前方本车道有阻塞障碍物，绝不向 d=0 右拉回正（防止借道中途往静止障碍物上撞！），保持当前或左侧安全偏移并以 -4.0 m/s^2 紧急制动；
    // 2) 仅当无阻塞障碍物且车辆已低速停滞时，才以 2.0 m/s 缓慢向车道内恢复。
    std::vector<TrajectoryPoint> backup;
    const double s0 = init_s[0];
    const double v0 = std::max(0.0, init_s[1]);
    const bool has_front_obs = pt_graph.HasBlockingInLaneObstacle();
    const bool creep_recover = (!has_front_obs && v0 < 1.0);
    const double decel = creep_recover ? 0.8 : -4.0;
    const double v_cap = 2.0;

    double target_d = std::min(std::max(init_d[0], -0.6), 2.85);
    if (has_front_obs && pt_graph.IsLeftBorrowLaneClear() && init_d[0] > 0.6) {
        // 借道中途收敛并保持在对向车道中心 d=2.75m（既不向右撞静止车，也不向左超界自锁）
        target_d = 2.75;
    } else if (creep_recover) {
        target_d = 0.0;
    }
    const lattice::QuinticPolynomialCurve1d lat_hold(
        init_d, std::array<double, 3>{{target_d, 0.0, 0.0}}, 10.0);

    for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += _cfg.output_time_resolution) {
        double s = s0;
        double v = 0.0;
        double a = 0.0;
        if (creep_recover) {
            const double t_acc = std::max(0.0, (v_cap - v0) / decel);
            if (t < t_acc) {
                v = v0 + decel * t;
                s = s0 + v0 * t + 0.5 * decel * t * t;
                a = decel;
            } else {
                v = v_cap;
                s = s0 + v0 * t_acc + 0.5 * decel * t_acc * t_acc + v_cap * (t - t_acc);
                a = 0.0;
            }
            s = std::min(ref_s.back(), s);
        } else {
            const double t_stop = (v0 > 1e-3) ? (v0 / (-decel)) : 0.0;
            const double tc = std::min(t, t_stop);
            s = std::min(ref_s.back(), s0 + v0 * tc + 0.5 * decel * tc * tc);
            v = (t < t_stop) ? std::max(0.0, v0 + decel * t) : 0.0;
            a = (t < t_stop) ? decel : 0.0;
        }
        const double rel_s = std::max(0.0, s - s0);
        const double d = (rel_s <= 10.0) ? lat_hold.Evaluate(0, rel_s) : target_d;
        const double dl = (rel_s <= 10.0) ? lat_hold.Evaluate(1, rel_s) : 0.0;

        const RefSample ref = SampleReferenceLineByS(reference_line, ref_s, s);
        const double one_minus_kd = std::max(0.2, 1.0 - ref.kappa * d);

        TrajectoryPoint pt;
        pt.x = ref.x - d * std::sin(ref.heading);
        pt.y = ref.y + d * std::cos(ref.heading);
        pt.heading = normalize_angle(ref.heading + std::atan2(dl, one_minus_kd));
        pt.kappa = ref.kappa;
        pt.v = v;
        pt.a_tau = a;
        pt.ax = a * std::cos(pt.heading);
        pt.ay = a * std::sin(pt.heading);
        pt.time_stamped = start_timestamp + t;
        backup.emplace_back(pt);
    }
    return backup;
}

void LatticePlanner::PopulateDebugData(
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
    bool used_backup)
{
    _debug_data.clear();
    _debug_data.timestamp = current_time;
    _debug_data.compute_time_ms = compute_ms;
    _debug_data.reference_speed = _reference_speed;
    _debug_data.ego_v = ego_state->v;
    _debug_data.path_qp_ok = !used_backup;
    _debug_data.speed_qp_ok = !used_backup;
    _debug_data.road_up_boundary = pt_graph.IsLeftBorrowLaneClear() ? 5.2 : 1.8;
    _debug_data.road_low_boundary = -1.8;

    {
        std::vector<FrenetPoint> ego_sl_vec;
        cartesion_set_to_frenet_set(
            vehicle_state_to_trajectory_point(ego_state), reference_line, ego_state, ego_sl_vec);
        if (!ego_sl_vec.empty()) {
            _debug_data.ego_sl = ego_sl_vec.front();
        }
    }

    _debug_data.start_sl.s = init_s[0];
    _debug_data.start_sl.s_dot = init_s[1];
    _debug_data.start_sl.s_dot_dot = init_s[2];
    _debug_data.start_sl.l = init_d[0];
    _debug_data.start_sl.l_prime = init_d[1];
    _debug_data.start_sl.l_prime_prime = init_d[2];

    for (const auto& st_obs : pt_graph.static_obstacles()) {
        FrenetPoint fp;
        fp.s = st_obs.s;
        fp.l = st_obs.l;
        _debug_data.static_obs_sl.push_back(fp);
    }
    for (const auto& dy_obs : pt_graph.dynamic_obstacles()) {
        FrenetPoint fp;
        fp.s = dy_obs.s;
        fp.l = dy_obs.l;
        fp.s_dot = dy_obs.s_dot;
        fp.l_dot = dy_obs.l_dot;
        _debug_data.dynamic_obs_sl.push_back(fp);
    }
    _debug_data.st_graph = pt_graph.ExportDebugSTGraph();

    if (best_lon && best_lat) {
        const double s_end = std::min(ref_s.back(), best_lon->Evaluate(0, _cfg.trajectory_time_length));
        _debug_data.path_length = std::max(0.0, s_end - init_s[0]);
        for (double s = init_s[0]; s <= s_end + 1e-6; s += 1.0) {
            const double rel_s = std::max(0.0, s - init_s[0]);
            FrenetPoint fp;
            fp.s = s;
            fp.l = best_lat->Evaluate(0, rel_s);
            fp.l_prime = best_lat->Evaluate(1, rel_s);
            fp.l_prime_prime = best_lat->Evaluate(2, rel_s);
            _debug_data.qp_path.push_back(fp);
        }
        for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += 0.1) {
            STPoint sp;
            sp.t = t;
            sp.s = std::max(0.0, best_lon->Evaluate(0, t) - init_s[0]);
            sp.s_dot = std::max(0.0, best_lon->Evaluate(1, t));
            sp.s_dot_dot = best_lon->Evaluate(2, t);
            _debug_data.qp_speed_profile.push_back(sp);
        }
    }
}

void LatticePlanner::plan(
    double current_time,
    const std::vector<PathPoint>& reference_line,
    const std::shared_ptr<VehicleState>& ego_state,
    const std::vector<derived_object_msgs::msg::Object>& obstacles,
    std::vector<TrajectoryPoint>& final_trajectory)
{
    const auto start_clock = std::chrono::steady_clock::now();
    if (reference_line.size() < 3 || !ego_state) {
        return;
    }

    // 1. 计算参考线弧长表（以主车投影点为 s=0，与 EMPlanner 保持完全一致的坐标系定义）
    const std::vector<double> ref_s = calculate_index_to_s(reference_line, ego_state);

    // 2. 通过共用 TrajectoryStitcher 计算规划起点并准备历史拼接轨迹
    const TrajectoryPoint planning_start_point =
        _stitcher.compute_planning_start_point(current_time, 0.1, ego_state);

    // 3. 计算 Frenet 坐标系三阶初始状态 (init_s, init_d)
    std::array<double, 3> init_s{{0.0, 0.0, 0.0}};
    std::array<double, 3> init_d{{0.0, 0.0, 0.0}};
    ComputeInitFrenetState(reference_line, ref_s, planning_start_point, &init_s, &init_d);

    // 4. 构建 PathTimeGraph 与规划目标（对前方急弯自动做舒适过弯限速）
    lattice::PathTimeGraph pt_graph(obstacles, reference_line, ref_s, ego_state, _cfg);

    double max_ahead_kappa = 0.0;
    for (double s_look = init_s[0]; s_look <= std::min(ref_s.back(), init_s[0] + 38.0); s_look += 2.0) {
        const RefSample rs = SampleReferenceLineByS(reference_line, ref_s, s_look);
        max_ahead_kappa = std::max(max_ahead_kappa, std::abs(rs.kappa));
    }
    lattice::PlanningTarget target;
    target.cruise_speed = _reference_speed;
    if (pt_graph.HasBlockingInLaneObstacle() || std::abs(init_d[0]) > 1.2) {
        // 借道绕行施工/违停车区间自动舒适限速至 7.0 m/s（约 25 km/h），保证近距离变道平顺且向心加速度留足余量
        target.cruise_speed = std::min(target.cruise_speed, 7.0);
    }
    if (max_ahead_kappa > 0.03) {
        // 弯道向心加速度舒适限速 v <= sqrt(a_lat_comfort / kappa)，确保 LQR/MPC 90° 过弯不外抛
        const double curve_v_limit = std::sqrt(1.5 / max_ahead_kappa);
        target.cruise_speed = std::max(3.2, std::min(target.cruise_speed, curve_v_limit));
    }
    if (ref_s.back() - init_s[0] < 18.0) {
        target.has_stop_point = true;
        target.stop_s = std::max(init_s[0], ref_s.back() - 2.0);
    }

    // 5. 生成一维横纵向多项式轨迹束
    lattice::Trajectory1dGenerator generator(init_s, init_d, pt_graph, _cfg);
    std::vector<std::shared_ptr<lattice::Curve1d>> lon_bundle;
    std::vector<std::shared_ptr<lattice::Curve1d>> lat_bundle;
    generator.GenerateTrajectoryBundles(target, &lon_bundle, &lat_bundle);

    // 6. 最小堆代价排序 + 惰性弹出校验六维运动学约束与 2D OBB 碰撞
    lattice::TrajectoryEvaluator evaluator(
        init_s, target, lon_bundle, lat_bundle, pt_graph, reference_line, ref_s, _cfg);

    // Town01 左侧有对向车道缓冲（15m 后严格上界 +3.35m），前 15m 恢复上界限制在 +3.75m 以远离左侧护栏
    const double recovery_margin_left = std::min(3.75, init_d[0] + std::max(0.35, init_d[1] * 3.0));
    const double left_bound_d = std::max(
        pt_graph.IsLeftBorrowLaneClear() ? 3.35 : 1.65,
        recovery_margin_left);
    const double right_bound_d = std::min(-1.25, init_d[0] - 0.25);

    std::vector<TrajectoryPoint> best_new_trajectory;
    std::shared_ptr<lattice::Curve1d> chosen_lon;
    std::shared_ptr<lattice::Curve1d> chosen_lat;
    bool found_valid = false;

    while (evaluator.has_more_pairs()) {
        auto pair = evaluator.next_top_pair();
        bool in_road = true;
        // 先以 0.1s 粗步长合成并完成运动学/碰撞筛选，通过后再仅对最优解做 0.02s 加密（提速约 5 倍）
        auto coarse = CombineTrajectory(
            reference_line, ref_s, *pair.first, *pair.second,
            planning_start_point.time_stamped, _cfg.trajectory_time_resolution,
            left_bound_d, right_bound_d, &in_road);
        if (!in_road) {
            continue;
        }
        if (!IsTrajectoryDynamicallyValid(coarse)) {
            continue;
        }
        if (!IsTrajectoryCollisionFree(coarse, planning_start_point.time_stamped, pt_graph)) {
            continue;
        }
        best_new_trajectory = CombineTrajectory(
            reference_line, ref_s, *pair.first, *pair.second,
            planning_start_point.time_stamped, _cfg.output_time_resolution,
            left_bound_d + 0.1, right_bound_d - 0.1, &in_road);
        chosen_lon = pair.first;
        chosen_lat = pair.second;
        found_valid = true;
        break;
    }

    // 7. 若所有候选均被约束或碰撞剪枝，启用 BackupTrajectoryGenerator 兜底减速
    if (!found_valid) {
        RCLCPP_WARN(LATTICE_LOG, "所有 Lattice 候选轨迹均未通过约束/碰撞检查，启用安全减速兜底轨迹");
        best_new_trajectory = GenerateBackupTrajectory(
            reference_line, ref_s, init_s, init_d, pt_graph, planning_start_point.time_stamped);
    }

    // 8. 拼接历史轨迹并填充 SL/ST 调试数据
    _stitcher.stitch_and_update(best_new_trajectory, final_trajectory);

    const double compute_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_clock).count();
    PopulateDebugData(
        current_time, compute_ms, reference_line, ref_s, ego_state,
        init_s, init_d, pt_graph, chosen_lon, chosen_lat, !found_valid);
}
