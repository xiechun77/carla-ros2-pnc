#include "planning_debug.h"

#include <cmath>
#include <cstdio>
#include <sstream>

void PlanningDebugData::clear()
{
    *this = PlanningDebugData();
}

namespace
{
// 只输出有限数，保留 3 位小数；NaN/Inf 输出 null，保证是合法 JSON
void put_num(std::ostringstream& os, double value)
{
    if (!std::isfinite(value))
    {
        os << "null";
        return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f", value);
    os << buf;
}

void put_key(std::ostringstream& os, const char* key)
{
    os << '"' << key << "\":";
}

void put_array(std::ostringstream& os, const std::vector<double>& values)
{
    os << '[';
    for (size_t i = 0; i < values.size(); i++)
    {
        if (i) os << ',';
        put_num(os, values[i]);
    }
    os << ']';
}

void put_bool(std::ostringstream& os, bool value)
{
    os << (value ? "true" : "false");
}

// {"s":[...],"l":[...]}
void put_sl_curve(std::ostringstream& os, const std::vector<FrenetPoint>& points)
{
    std::vector<double> s, l;
    s.reserve(points.size());
    l.reserve(points.size());
    for (const auto& p : points)
    {
        s.push_back(p.s);
        l.push_back(p.l);
    }
    os << '{';
    put_key(os, "s"); put_array(os, s); os << ',';
    put_key(os, "l"); put_array(os, l);
    os << '}';
}

// [{"s":..,"l":..,"s_dot":..,"l_dot":..}, ...]
void put_obstacles(std::ostringstream& os, const std::vector<FrenetPoint>& points)
{
    os << '[';
    for (size_t i = 0; i < points.size(); i++)
    {
        if (i) os << ',';
        os << '{';
        put_key(os, "s"); put_num(os, points[i].s); os << ',';
        put_key(os, "l"); put_num(os, points[i].l); os << ',';
        put_key(os, "s_dot"); put_num(os, points[i].s_dot); os << ',';
        put_key(os, "l_dot"); put_num(os, points[i].l_dot);
        os << '}';
    }
    os << ']';
}

// {"t":[...],"s":[...],"v":[...],"a":[...]}
void put_st_curve(std::ostringstream& os, const std::vector<STPoint>& points)
{
    std::vector<double> t, s, v, a;
    for (const auto& p : points)
    {
        t.push_back(p.t);
        s.push_back(p.s);
        v.push_back(p.s_dot);
        a.push_back(p.s_dot_dot);
    }
    os << '{';
    put_key(os, "t"); put_array(os, t); os << ',';
    put_key(os, "s"); put_array(os, s); os << ',';
    put_key(os, "v"); put_array(os, v); os << ',';
    put_key(os, "a"); put_array(os, a);
    os << '}';
}
} // namespace

std::string planning_debug_to_json(const PlanningDebugData& d)
{
    std::ostringstream os;
    os << '{';
    put_key(os, "version"); os << 1 << ',';
    put_key(os, "t"); put_num(os, d.timestamp); os << ',';
    put_key(os, "compute_ms"); put_num(os, d.compute_time_ms); os << ',';
    put_key(os, "ref_speed"); put_num(os, d.reference_speed); os << ',';
    put_key(os, "ego_v"); put_num(os, d.ego_v); os << ',';
    put_key(os, "path_qp_ok"); put_bool(os, d.path_qp_ok); os << ',';
    put_key(os, "speed_qp_ok"); put_bool(os, d.speed_qp_ok); os << ',';

    // ---------------- SL ----------------
    put_key(os, "sl"); os << '{';
    put_key(os, "road_up"); put_num(os, d.road_up_boundary); os << ',';
    put_key(os, "road_low"); put_num(os, d.road_low_boundary); os << ',';
    put_key(os, "obs_length"); put_num(os, d.obs_length); os << ',';
    put_key(os, "obs_width"); put_num(os, d.obs_width); os << ',';
    put_key(os, "ego"); os << '['; put_num(os, d.ego_sl.s); os << ','; put_num(os, d.ego_sl.l); os << "],";
    put_key(os, "start"); os << '['; put_num(os, d.start_sl.s); os << ','; put_num(os, d.start_sl.l); os << "],";
    put_key(os, "dp_sample_s"); put_array(os, d.dp_sample_s); os << ',';
    put_key(os, "dp_sample_l"); put_array(os, d.dp_sample_l); os << ',';
    put_key(os, "dp_path"); put_sl_curve(os, d.dp_path); os << ',';
    put_key(os, "dense_dp_path"); put_sl_curve(os, d.dense_dp_path); os << ',';
    put_key(os, "l_min"); put_array(os, d.path_l_min); os << ',';
    put_key(os, "l_max"); put_array(os, d.path_l_max); os << ',';
    put_key(os, "qp_path"); put_sl_curve(os, d.qp_path); os << ',';
    put_key(os, "static_obs"); put_obstacles(os, d.static_obs_sl); os << ',';
    put_key(os, "dynamic_obs"); put_obstacles(os, d.dynamic_obs_sl);
    os << "},";

    // ---------------- ST ----------------
    put_key(os, "st"); os << '{';
    put_key(os, "path_length"); put_num(os, d.path_length); os << ',';
    put_key(os, "sample_t"); put_array(os, d.st_sample_t); os << ',';
    put_key(os, "sample_s"); put_array(os, d.st_sample_s); os << ',';
    put_key(os, "dp"); put_st_curve(os, d.dp_speed_profile); os << ',';
    put_key(os, "qp"); put_st_curve(os, d.qp_speed_profile); os << ',';
    put_key(os, "s_lb"); put_array(os, d.s_lb); os << ',';
    put_key(os, "s_ub"); put_array(os, d.s_ub); os << ',';
    put_key(os, "v_lb"); put_array(os, d.s_dot_lb); os << ',';
    put_key(os, "v_ub"); put_array(os, d.s_dot_ub); os << ',';
    put_key(os, "obs_on_path"); put_obstacles(os, d.dynamic_obs_on_path); os << ',';
    put_key(os, "st_graph"); os << '[';
    for (size_t i = 0; i < d.st_graph.size(); i++)
    {
        if (i) os << ',';
        const auto& node = d.st_graph[i];
        os << '{';
        bool first = true;
        for (const char* key : {"t_in", "t_out", "s_in", "s_out"})
        {
            auto it = node.find(key);
            if (it == node.end()) continue;
            if (!first) os << ',';
            first = false;
            put_key(os, key); put_num(os, it->second);
        }
        os << '}';
    }
    os << ']';
    os << '}';

    os << '}';
    return os.str();
}
