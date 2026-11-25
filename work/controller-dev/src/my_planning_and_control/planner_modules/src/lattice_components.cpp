#include "lattice_components.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lattice {

namespace {

// 在参考线上按弧长 s 线性插值获取参考点曲率
double MatchKappaByS(
    const std::vector<PathPoint>& ref_line,
    const std::vector<double>& ref_s,
    double s)
{
    if (ref_line.empty() || ref_s.empty()) {
        return 0.0;
    }
    if (s <= ref_s.front()) {
        return ref_line.front().kappa;
    }
    if (s >= ref_s.back()) {
        return ref_line.back().kappa;
    }
    auto it = std::upper_bound(ref_s.begin(), ref_s.end(), s);
    size_t idx = static_cast<size_t>(std::distance(ref_s.begin(), it));
    if (idx == 0) {
        return ref_line.front().kappa;
    }
    const size_t i0 = idx - 1;
    const size_t i1 = std::min(idx, ref_line.size() - 1);
    const double ds = ref_s[i1] - ref_s[i0];
    if (ds <= 1e-6) {
        return ref_line[i0].kappa;
    }
    const double ratio = (s - ref_s[i0]) / ds;
    const double raw_k = ref_line[i0].kappa + ratio * (ref_line[i1].kappa - ref_line[i0].kappa);
    return std::min(std::max(raw_k, -0.32), 0.32);
}

// 将单个世界坐标点投影到参考线，输出 (s, l, ref_heading)
void ProjectToRefLine(
    const std::vector<PathPoint>& ref_line,
    const std::vector<double>& ref_s,
    double x,
    double y,
    double* out_s,
    double* out_l,
    double* out_ref_heading)
{
    size_t best_i = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < ref_line.size(); ++i) {
        const double dx = x - ref_line[i].x;
        const double dy = y - ref_line[i].y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_i = i;
        }
    }
    const auto& mp = ref_line[best_i];
    const double cos_h = std::cos(mp.heading);
    const double sin_h = std::sin(mp.heading);
    const double dx = x - mp.x;
    const double dy = y - mp.y;
    const double tau_proj = dx * cos_h + dy * sin_h;
    const double ref_h = mp.heading + mp.kappa * tau_proj;
    const double proj_x = mp.x + tau_proj * cos_h;
    const double proj_y = mp.y + tau_proj * sin_h;
    const double nor_x = -std::sin(ref_h);
    const double nor_y = std::cos(ref_h);

    *out_s = ref_s[best_i] + tau_proj;
    *out_l = (x - proj_x) * nor_x + (y - proj_y) * nor_y;
    *out_ref_heading = ref_h;
}

}  // namespace

PathTimeGraph::PathTimeGraph(
    const std::vector<derived_object_msgs::msg::Object>& raw_obstacles,
    const std::vector<PathPoint>& reference_line,
    const std::vector<double>& ref_s,
    const std::shared_ptr<VehicleState>& ego_state,
    const LatticeConfig& cfg)
{
    if (reference_line.empty() || ref_s.empty() || !ego_state) {
        return;
    }

    // 检查自车当前所处参考线段是否已驶出急弯（急弯内不提前横向借道，先完成本车道过弯）
    double ego_s = 0.0, ego_l = 0.0, ego_rh = 0.0;
    ProjectToRefLine(reference_line, ref_s, ego_state->x, ego_state->y, &ego_s, &ego_l, &ego_rh);
    double near_max_kappa = 0.0;
    for (double sk = ego_s; sk <= std::min(ref_s.back(), ego_s + 10.0); sk += 2.0) {
        near_max_kappa = std::max(near_max_kappa, std::abs(MatchKappaByS(reference_line, ref_s, sk)));
    }
    const bool in_sharp_turn = (near_max_kappa > 0.05);

    for (const auto& obj : raw_obstacles) {
        if (obj.id == ego_state->id) {
            continue;
        }
        const double dx_ego = obj.pose.position.x - ego_state->x;
        const double dy_ego = obj.pose.position.y - ego_state->y;
        if (std::hypot(dx_ego, dy_ego) < 0.3) {
            continue;
        }

        double s_obs = 0.0;
        double l_obs = 0.0;
        double ref_h = 0.0;
        ProjectToRefLine(reference_line, ref_s, obj.pose.position.x, obj.pose.position.y,
                         &s_obs, &l_obs, &ref_h);

        // 过滤超出规划关心范围的远端障碍物
        if (s_obs < -12.0 || s_obs > 70.0 || std::abs(l_obs) > 20.0) {
            continue;
        }

        ObstacleSLBox box;
        box.id = obj.id;
        box.x = obj.pose.position.x;
        box.y = obj.pose.position.y;
        tf2::Quaternion tf_q;
        tf2::fromMsg(obj.pose.orientation, tf_q);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        box.heading = yaw;

        box.vx = obj.twist.linear.x;
        box.vy = obj.twist.linear.y;
        box.speed = std::hypot(box.vx, box.vy);
        if (obj.shape.dimensions.size() >= 2 &&
            obj.shape.dimensions[0] > 0.2 && obj.shape.dimensions[1] > 0.2) {
            box.length = obj.shape.dimensions[0];
            box.width = obj.shape.dimensions[1];
        } else {
            // 行人与车辆默认包络尺寸区分
            box.length = (box.speed > 0.1 && box.speed < 2.8) ? 1.0 : 4.6;
            box.width = (box.speed > 0.1 && box.speed < 2.8) ? 1.0 : 2.0;
        }

        box.s = s_obs;
        box.l = l_obs;
        const double cos_r = std::cos(ref_h);
        const double sin_r = std::sin(ref_h);
        box.s_dot = box.vx * cos_r + box.vy * sin_r;
        box.l_dot = -box.vx * sin_r + box.vy * cos_r;
        box.is_static = (box.speed < 0.15);

        _obstacles.push_back(box);
        if (box.is_static) {
            _static_obstacles.push_back(box);
            // 注意两车半车长之和约 4.7m，下限取 -7.5m 确保自车后保险杠完全越过静止车前保险杠后才解除借道保持
            if (!in_sharp_turn && box.s > -7.5 && box.s < 55.0 && std::abs(box.l) < 2.0) {
                _has_blocking_in_lane_obs = true;
            }
            if (box.l > 1.6 && box.l < 5.0 && box.s > -6.0 && box.s < 45.0) {
                _left_lane_clear = false;
            }
        } else {
            _dynamic_obstacles.push_back(box);
            if (box.l > 1.6 && box.l < 5.5 && box.s > -10.0 && box.s < 50.0 && box.s_dot < -0.5) {
                _left_lane_clear = false;
            }
            if (!in_sharp_turn && std::abs(box.l) < 1.8 && box.s > -7.5 && box.s < 45.0 &&
                box.s_dot >= -0.2 && box.s_dot < 6.5 && std::abs(box.l_dot) < 0.6) {
                _has_blocking_in_lane_obs = true;
            }
        }
    }

    // 为动态障碍物构建 ST 图投影区间
    const double lat_conflict_half = 0.5 * (cfg.ego_width + 2.0) + cfg.dynamic_lat_buffer;
    for (const auto& dyn : _dynamic_obstacles) {
        bool entered = false;
        double t_in = 0.0, t_out = 0.0, s_in = 0.0, s_out = 0.0;
        for (double t = 0.0; t <= cfg.trajectory_time_length + 1e-6; t += cfg.trajectory_time_resolution) {
            const double pred_x = dyn.x + dyn.vx * t;
            const double pred_y = dyn.y + dyn.vy * t;
            double ps = 0.0, pl = 0.0, ph = 0.0;
            ProjectToRefLine(reference_line, ref_s, pred_x, pred_y, &ps, &pl, &ph);
            if (std::abs(pl) <= lat_conflict_half && ps >= -4.0 && ps <= ref_s.back() + 5.0) {
                if (!entered) {
                    entered = true;
                    t_in = t;
                    s_in = ps;
                }
                t_out = t;
                s_out = ps;
            } else if (entered) {
                break;
            }
        }
        if (entered && t_out >= t_in) {
            STObstacleInterval st;
            st.id = dyn.id;
            st.t_in = t_in;
            st.t_out = std::max(t_out, t_in + 0.2);
            st.s_in = s_in;
            st.s_out = s_out;
            st.v_lon = dyn.s_dot;
            _st_intervals.push_back(st);
        }
    }
}

std::vector<std::vector<std::pair<double, double>>> PathTimeGraph::GetPathBlockingIntervals(
    double t_start, double t_end, double dt) const
{
    std::vector<std::vector<std::pair<double, double>>> result;
    const double half_s = 3.5;
    for (double t = t_start; t <= t_end + 1e-6; t += dt) {
        std::vector<std::pair<double, double>> slice;
        for (const auto& st : _st_intervals) {
            if (t >= st.t_in && t <= st.t_out) {
                const double ratio = (st.t_out > st.t_in + 1e-4)
                    ? ((t - st.t_in) / (st.t_out - st.t_in))
                    : 0.0;
                const double s_mid = st.s_in + ratio * (st.s_out - st.s_in);
                slice.emplace_back(s_mid - half_s, s_mid + half_s);
            }
        }
        result.push_back(std::move(slice));
    }
    return result;
}

std::vector<std::unordered_map<std::string, double>> PathTimeGraph::ExportDebugSTGraph() const
{
    std::vector<std::unordered_map<std::string, double>> out;
    out.reserve(_st_intervals.size());
    for (const auto& st : _st_intervals) {
        out.push_back({
            {"t_in", st.t_in},
            {"t_out", st.t_out},
            {"s_in", st.s_in},
            {"s_out", st.s_out},
        });
    }
    return out;
}

// ======================= Trajectory1dGenerator =======================

Trajectory1dGenerator::Trajectory1dGenerator(
    const std::array<double, 3>& init_s,
    const std::array<double, 3>& init_d,
    const PathTimeGraph& pt_graph,
    const LatticeConfig& cfg)
    : _init_s(init_s), _init_d(init_d), _pt_graph(pt_graph), _cfg(cfg)
{
}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& target,
    std::vector<std::shared_ptr<Curve1d>>* lon_bundle,
    std::vector<std::shared_ptr<Curve1d>>* lat_bundle) const
{
    lon_bundle->clear();
    lat_bundle->clear();
    GenerateLongitudinalBundle(target, lon_bundle);
    GenerateLateralBundle(lat_bundle);
}

void Trajectory1dGenerator::GenerateLongitudinalBundle(
    const PlanningTarget& target,
    std::vector<std::shared_ptr<Curve1d>>* lon_bundle) const
{
    // 1. 巡航速度采样（四次多项式）
    const std::array<double, 5> time_samples{{1.0, 2.0, 3.0, 4.0, _cfg.trajectory_time_length}};
    const double v_cruise = std::max(1.5, target.cruise_speed);
    const std::array<double, 5> vel_ratios{{0.4, 0.6, 0.78, 0.9, 1.0}};

    for (const double t : time_samples) {
        const double v_upper = std::min(_cfg.max_lon_vel, _init_s[1] + _cfg.max_lon_acc * t);
        const double v_lower = std::max(0.0, _init_s[1] + _cfg.min_lon_acc * t);

        for (const double ratio : vel_ratios) {
            const double v_end = std::min(v_upper, std::max(v_lower, v_cruise * ratio));
            auto base = std::make_shared<QuarticPolynomialCurve1d>(
                _init_s, std::array<double, 2>{{v_end, 0.0}}, t);
            auto traj = std::make_shared<LatticeTrajectory1d>(base);
            traj->set_target_velocity(v_end);
            traj->set_target_time(t);
            lon_bundle->push_back(traj);
        }
        // 仅当存在动态 ST 冲突或终点停车目标时才加入零速制动候选，防止在可借道绕行的静态车前主动刹停等待
        if ((!_pt_graph.st_intervals().empty() || target.has_stop_point) && v_lower <= 0.5) {
            auto base = std::make_shared<QuarticPolynomialCurve1d>(
                _init_s, std::array<double, 2>{{0.0, 0.0}}, t);
            auto traj = std::make_shared<LatticeTrajectory1d>(base);
            traj->set_target_velocity(0.0);
            traj->set_target_time(t);
            lon_bundle->push_back(traj);
        }
    }

    // 2. 定点停车采样（五次多项式）
    if (target.has_stop_point) {
        const double s_stop = std::max(_init_s[0], target.stop_s);
        for (const double t : {1.5, 2.5, 3.5, 5.0}) {
            auto base = std::make_shared<QuinticPolynomialCurve1d>(
                _init_s, std::array<double, 3>{{s_stop, 0.0, 0.0}}, t);
            auto traj = std::make_shared<LatticeTrajectory1d>(base);
            traj->set_target_position(s_stop);
            traj->set_target_velocity(0.0);
            traj->set_target_time(t);
            lon_bundle->push_back(traj);
        }
    }

    // 3. ST 动态障碍物跟车（Yield/Follow）与超车（Overtake）采样（五次多项式）
    for (const auto& st : _pt_graph.st_intervals()) {
        for (const double t_sample : {st.t_in, 0.5 * (st.t_in + st.t_out), st.t_out}) {
            if (t_sample < 0.6 || t_sample > _cfg.trajectory_time_length) {
                continue;
            }
            const double ratio = (st.t_out > st.t_in + 1e-4)
                ? ((t_sample - st.t_in) / (st.t_out - st.t_in))
                : 0.0;
            const double s_obs = st.s_in + ratio * (st.s_out - st.s_in);

            // 跟车/让行终点状态
            const double s_follow = s_obs - _cfg.lon_yield_buffer;
            if (s_follow >= _init_s[0] - 0.5) {
                const double v_follow = std::max(0.0, std::min(v_cruise, st.v_lon));
                auto base_f = std::make_shared<QuinticPolynomialCurve1d>(
                    _init_s, std::array<double, 3>{{std::max(_init_s[0], s_follow), v_follow, 0.0}}, t_sample);
                auto traj_f = std::make_shared<LatticeTrajectory1d>(base_f);
                traj_f->set_target_position(s_follow);
                traj_f->set_target_velocity(v_follow);
                traj_f->set_target_time(t_sample);
                lon_bundle->push_back(traj_f);
            }

            // 超车/抢行终点状态
            const double s_overtake = s_obs + _cfg.lon_overtake_buffer;
            const double s_max_reach = _init_s[0] + _init_s[1] * t_sample + 0.5 * _cfg.max_lon_acc * t_sample * t_sample;
            if (s_overtake <= s_max_reach) {
                const double v_overtake = std::min(_cfg.max_lon_vel, std::max(v_cruise, st.v_lon + 2.0));
                auto base_o = std::make_shared<QuinticPolynomialCurve1d>(
                    _init_s, std::array<double, 3>{{s_overtake, v_overtake, 0.0}}, t_sample);
                auto traj_o = std::make_shared<LatticeTrajectory1d>(base_o);
                traj_o->set_target_position(s_overtake);
                traj_o->set_target_velocity(v_overtake);
                traj_o->set_target_time(t_sample);
                lon_bundle->push_back(traj_o);
            }
        }
    }
    // 4. 仅在对向车道被占用、无法借道绕行时，才生成静态障碍物前定点停车候选
    if (!_pt_graph.IsLeftBorrowLaneClear()) {
        for (const auto& st_obs : _pt_graph.static_obstacles()) {
            if (st_obs.s > 3.0 && st_obs.s < 45.0 && std::abs(st_obs.l) < 2.0) {
                const double s_stop = std::max(_init_s[0], st_obs.s - 5.5);
                for (const double t_stop : {2.5, 3.5, 5.0}) {
                    auto base_s = std::make_shared<QuinticPolynomialCurve1d>(
                        _init_s, std::array<double, 3>{{s_stop, 0.0, 0.0}}, t_stop);
                    auto traj_s = std::make_shared<LatticeTrajectory1d>(base_s);
                    traj_s->set_target_position(s_stop);
                    traj_s->set_target_velocity(0.0);
                    traj_s->set_target_time(t_stop);
                    lon_bundle->push_back(traj_s);
                }
            }
        }
    }
}

void Trajectory1dGenerator::GenerateLateralBundle(
    std::vector<std::shared_ptr<Curve1d>>* lat_bundle) const
{
    // 固定多尺度纵向跨度（包含 6m / 9m 近距离敏捷绕障跨度）
    std::vector<double> end_s_candidates = {6.0, 9.0, 13.0, 18.0, 26.0, 38.0};

    // 障碍物前缘锚定动态跨度（工业界 Lattice 定点绕障核心技巧）：
    // 直接以距前方本车道阻塞障碍物前缘 Δs_front 为五次多项式终点，使车辆在到达障碍物前恰好完成横移并平行掠过
    const double safety_front_gap = 0.5 * _cfg.ego_length + 2.3 + _cfg.static_lon_buffer + 0.4;
    for (const auto& obs : _pt_graph.obstacles()) {
        if (std::abs(obs.l) < 2.1 && obs.s > _init_s[0] + 1.0 && obs.s < _init_s[0] + 45.0) {
            const double ds_front = std::max(4.5, (obs.s - _init_s[0]) - safety_front_gap);
            const double ds_mid = std::max(5.5, (obs.s - _init_s[0]) - 2.0);
            end_s_candidates.push_back(ds_front);
            end_s_candidates.push_back(ds_mid);
        }
    }

    std::vector<double> end_d_candidates = {0.0, -0.2, 0.35, 0.7};

    // Town01 对向车道安全甜区位于 d ∈ [2.45, 3.08]m（既留足右侧静态车安全余量，又远离左侧马路牙子）
    if (_pt_graph.HasBlockingInLaneObstacle() && _pt_graph.IsLeftBorrowLaneClear()) {
        for (const double d_borrow : {2.45, 2.78, 3.05}) {
            end_d_candidates.push_back(d_borrow);
        }
    } else if (std::abs(_init_d[0]) > 0.9) {
        for (const double d_hold : {1.6, 2.45, 2.90}) {
            end_d_candidates.push_back(d_hold);
        }
    }

    for (const double s_len : end_s_candidates) {
        for (const double d_end : end_d_candidates) {
            auto base = std::make_shared<QuinticPolynomialCurve1d>(
                _init_d, std::array<double, 3>{{d_end, 0.0, 0.0}}, s_len);
            lat_bundle->push_back(std::make_shared<LatticeTrajectory1d>(base));
        }
    }
}

// ======================= TrajectoryEvaluator =======================

TrajectoryEvaluator::TrajectoryEvaluator(
    const std::array<double, 3>& init_s,
    const PlanningTarget& target,
    const std::vector<PtrCurve1d>& lon_trajectories,
    const std::vector<PtrCurve1d>& lat_trajectories,
    const PathTimeGraph& pt_graph,
    const std::vector<PathPoint>& reference_line,
    const std::vector<double>& ref_s,
    const LatticeConfig& cfg)
    : _init_s(init_s),
      _pt_graph(pt_graph),
      _reference_line(reference_line),
      _ref_s(ref_s),
      _cfg(cfg)
{
    _blocking_intervals = _pt_graph.GetPathBlockingIntervals(
        0.0, _cfg.trajectory_time_length, _cfg.trajectory_time_resolution);

    // 构造引导速度序列
    for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += _cfg.trajectory_time_resolution) {
        if (target.has_stop_point) {
            const double dist = std::max(0.0, target.stop_s - _init_s[0]);
            const double v_guide = std::min(target.cruise_speed, std::sqrt(2.0 * 1.8 * dist));
            _ref_s_dots.push_back(v_guide);
        } else {
            _ref_s_dots.push_back(target.cruise_speed);
        }
    }

    // 过滤不满足一维纵向硬约束的轨迹，并将可行 (lon, lat) 组合推入最小堆
    for (const auto& lon_traj : lon_trajectories) {
        if (!IsValidLongitudinal1d(*lon_traj)) {
            continue;
        }
        for (const auto& lat_traj : lat_trajectories) {
            const double cost = EvaluateCost(target, lon_traj, lat_traj);
            if (std::isfinite(cost)) {
                _cost_queue.emplace(Pair(lon_traj, lat_traj), cost);
            }
        }
    }
}

TrajectoryEvaluator::Pair TrajectoryEvaluator::next_top_pair()
{
    auto top = _cost_queue.top();
    _cost_queue.pop();
    return top.first;
}

bool TrajectoryEvaluator::IsValidLongitudinal1d(const Curve1d& lon_traj) const
{
    double prev_s = lon_traj.Evaluate(0, 0.0);
    for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += _cfg.trajectory_time_resolution) {
        const double s = lon_traj.Evaluate(0, t);
        const double v = lon_traj.Evaluate(1, t);
        const double a = lon_traj.Evaluate(2, t);
        const double j = lon_traj.Evaluate(3, t);
        if (v < _cfg.min_lon_vel || v > _cfg.max_lon_vel) {
            return false;
        }
        if (a < _cfg.min_lon_acc || a > _cfg.max_lon_acc) {
            return false;
        }
        if (std::abs(j) > _cfg.max_lon_jerk) {
            return false;
        }
        if (s + 1e-3 < prev_s) {
            return false;
        }
        prev_s = s;
    }
    return true;
}

double TrajectoryEvaluator::EvaluateCost(
    const PlanningTarget& target,
    const PtrCurve1d& lon_traj,
    const PtrCurve1d& lat_traj) const
{
    const double lon_obj = LonObjectiveCost(lon_traj, target);
    const double lon_jerk = LonComfortCost(lon_traj);
    const double lon_coll = LonCollisionCost(lon_traj);
    const double cent_acc = CentripetalAccCost(lon_traj);

    const double eval_s_len = std::max(
        10.0,
        std::min(55.0, lon_traj->Evaluate(0, _cfg.trajectory_time_length) - _init_s[0]));
    const double lat_off = LatOffsetCost(lat_traj, eval_s_len);
    const double lat_comf = LatComfortCost(lon_traj, lat_traj);

    return lon_obj * _cfg.weight_lon_objective +
           lon_jerk * _cfg.weight_lon_jerk +
           lon_coll * _cfg.weight_lon_collision +
           cent_acc * _cfg.weight_centripetal_acc +
           lat_off * _cfg.weight_lat_offset +
           lat_comf * _cfg.weight_lat_comfort;
}

double TrajectoryEvaluator::LonObjectiveCost(
    const PtrCurve1d& lon_traj,
    const PlanningTarget& /*target*/) const
{
    const double dist_s = std::max(
        0.0, lon_traj->Evaluate(0, _cfg.trajectory_time_length) - lon_traj->Evaluate(0, 0.0));
    double speed_err_sum = 0.0;
    double weight_sum = 0.0;
    for (size_t i = 0; i < _ref_s_dots.size(); ++i) {
        const double t = static_cast<double>(i) * _cfg.trajectory_time_resolution;
        const double w = t * t + 0.1;
        speed_err_sum += w * std::abs(_ref_s_dots[i] - lon_traj->Evaluate(1, t));
        weight_sum += w;
    }
    const double speed_cost = speed_err_sum / (weight_sum + 1e-6);
    const double dist_cost = 1.0 / (1.0 + dist_s);
    return (speed_cost * _cfg.weight_target_speed + dist_cost * _cfg.weight_dist_travelled) /
           (_cfg.weight_target_speed + _cfg.weight_dist_travelled);
}

double TrajectoryEvaluator::LonComfortCost(const PtrCurve1d& lon_traj) const
{
    double sqr_sum = 0.0;
    double abs_sum = 0.0;
    for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += _cfg.trajectory_time_resolution) {
        const double c = lon_traj->Evaluate(3, t) / _cfg.max_lon_jerk;
        sqr_sum += c * c;
        abs_sum += std::abs(c);
    }
    return sqr_sum / (abs_sum + 1e-6);
}

double TrajectoryEvaluator::LonCollisionCost(const PtrCurve1d& lon_traj) const
{
    double sqr_sum = 0.0;
    double abs_sum = 0.0;
    const double sigma = 2.0;
    for (size_t i = 0; i < _blocking_intervals.size(); ++i) {
        const auto& intervals = _blocking_intervals[i];
        if (intervals.empty()) {
            continue;
        }
        const double t = static_cast<double>(i) * _cfg.trajectory_time_resolution;
        const double s = lon_traj->Evaluate(0, t);
        for (const auto& iv : intervals) {
            double dist = 0.0;
            if (s < iv.first - _cfg.lon_yield_buffer) {
                dist = iv.first - _cfg.lon_yield_buffer - s;
            } else if (s > iv.second + _cfg.lon_overtake_buffer) {
                dist = s - iv.second - _cfg.lon_overtake_buffer;
            }
            const double c = std::exp(-dist * dist / (2.0 * sigma * sigma));
            sqr_sum += c * c;
            abs_sum += c;
        }
    }
    return sqr_sum / (abs_sum + 1e-6);
}

double TrajectoryEvaluator::CentripetalAccCost(const PtrCurve1d& lon_traj) const
{
    double sum = 0.0;
    double sqr_sum = 0.0;
    for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += _cfg.trajectory_time_resolution) {
        const double s = lon_traj->Evaluate(0, t);
        const double v = lon_traj->Evaluate(1, t);
        const double kappa = MatchKappaByS(_reference_line, _ref_s, s);
        const double ay = v * v * kappa;
        sum += std::abs(ay);
        sqr_sum += ay * ay;
    }
    return sqr_sum / (sum + 1e-6);
}

double TrajectoryEvaluator::LatOffsetCost(const PtrCurve1d& lat_traj, double eval_s_len) const
{
    const double d0 = lat_traj->Evaluate(0, 0.0);
    double sqr_sum = 0.0;
    double abs_sum = 0.0;
    for (double s = 0.0; s <= eval_s_len + 1e-6; s += _cfg.space_resolution) {
        const double d = lat_traj->Evaluate(0, s);
        const double c = d / _cfg.lat_offset_bound;
        const double w = (d * d0 < 0.0) ? 10.0 : 1.0;  // 抑制跨过中心线的反向过冲
        sqr_sum += c * c * w;
        abs_sum += std::abs(c) * w;
    }
    return sqr_sum / (abs_sum + 1e-6);
}

double TrajectoryEvaluator::LatComfortCost(
    const PtrCurve1d& lon_traj,
    const PtrCurve1d& lat_traj) const
{
    double max_lat_acc = 0.0;
    for (double t = 0.0; t <= _cfg.trajectory_time_length + 1e-6; t += _cfg.trajectory_time_resolution) {
        const double s = lon_traj->Evaluate(0, t);
        const double s_dot = lon_traj->Evaluate(1, t);
        const double s_ddot = lon_traj->Evaluate(2, t);
        const double rel_s = std::max(0.0, s - _init_s[0]);
        const double dl = lat_traj->Evaluate(1, rel_s);
        const double ddl = lat_traj->Evaluate(2, rel_s);
        const double cost = std::abs(ddl * s_dot * s_dot + dl * s_ddot);
        if (cost > max_lat_acc) {
            max_lat_acc = cost;
        }
    }
    return max_lat_acc / std::max(1.0, _cfg.max_lat_acc);
}

}  // namespace lattice
