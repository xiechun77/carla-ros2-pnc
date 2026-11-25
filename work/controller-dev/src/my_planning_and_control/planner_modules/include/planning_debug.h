#pragma once
// EM planner 单周期调试数据（只用于可视化/记录，不参与规划计算）
// planning_agent 将其序列化为 JSON，发布到 /carla/ego_vehicle/planning_debug
#include <string>
#include <vector>
#include <unordered_map>
#include "common.h"

struct PlanningDebugData
{
    // ---------- 通用 ----------
    double timestamp = 0.0;        // 规划时刻（与轨迹 time_stamped 同一时钟）
    double compute_time_ms = 0.0;  // 本周期 EMPlanner::planning_run_step 耗时
    double reference_speed = 0.0;  // 参考速度 m/s
    double ego_v = 0.0;            // 主车实际速度 m/s
    bool path_qp_ok = false;       // 路径 QP 是否求解成功
    bool speed_qp_ok = false;      // 速度 QP 是否求解成功

    // ---------- SL 图（以参考线为 s 轴，s=0 为主车投影点） ----------
    double road_up_boundary = 0.0;
    double road_low_boundary = 0.0;
    double obs_length = 5.0;       // 与路径凸空间中假定的障碍物尺寸一致
    double obs_width = 3.0;
    FrenetPoint ego_sl;            // 主车
    FrenetPoint start_sl;          // 规划起点
    std::vector<double> dp_sample_s;   // DP 采样列的 s
    std::vector<double> dp_sample_l;   // DP 每列的 l 取值
    std::vector<FrenetPoint> dp_path;        // DP 最优路径（粗）
    std::vector<FrenetPoint> dense_dp_path;  // 加密后的 DP 路径
    std::vector<double> path_l_min;          // 凸空间下界（与 dense_dp_path 对齐）
    std::vector<double> path_l_max;          // 凸空间上界
    std::vector<FrenetPoint> qp_path;        // QP 路径
    std::vector<FrenetPoint> static_obs_sl;  // 静态障碍物（参考线投影）
    std::vector<FrenetPoint> dynamic_obs_sl; // 动态障碍物（参考线投影，仅显示，路径规划不使用）

    // ---------- ST 图（以本周期规划路径为 s 轴，s=0 为规划起点） ----------
    double path_length = 0.0;                  // 规划路径长度，ST 图 s 上界
    std::vector<double> st_sample_t;           // 速度 DP 采样的 t
    std::vector<double> st_sample_s;           // 速度 DP 采样的 s
    std::vector<STPoint> dp_speed_profile;     // 速度 DP 结果
    std::vector<double> s_lb, s_ub, s_dot_lb, s_dot_ub; // 速度 QP 凸空间（与 dp_speed_profile 对齐）
    std::vector<STPoint> qp_speed_profile;     // 速度 QP 结果（未加密）
    std::vector<FrenetPoint> dynamic_obs_on_path; // 动态障碍物在规划路径上的投影（ST 图的输入）
    std::vector<std::unordered_map<std::string, double>> st_graph; // 进入 ST 图的障碍物 t_in/t_out/s_in/s_out

    void clear();
};

// 序列化为紧凑 JSON（非有限数写成 null）
std::string planning_debug_to_json(const PlanningDebugData& data);
