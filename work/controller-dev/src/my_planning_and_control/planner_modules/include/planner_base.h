#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common.h"
#include "derived_object_msgs/msg/object.hpp"
#include "planning_debug.h"

/**
 * @brief 规划器统一抽象基类（纯 C++，不继承 rclcpp::Node）
 *
 * 所有规划算法（EMPlanner、LatticePlanner 等）均实现此接口，
 * 以便 PlanningAgent 通过参数 planner_type 统一调度与公平对比。
 */
class PlannerBase
{
public:
    virtual ~PlannerBase() = default;

    // 规划器标识名（如 "em", "lattice"）
    virtual std::string name() const = 0;

    // 单步规划入口
    virtual void plan(
        double current_time,
        const std::vector<PathPoint>& reference_line,
        const std::shared_ptr<VehicleState>& ego_state,
        const std::vector<derived_object_msgs::msg::Object>& obstacles,
        std::vector<TrajectoryPoint>& final_trajectory) = 0;

    // 获取最近一次规划周期的调试数据（只读）
    virtual const PlanningDebugData& debug_data() const = 0;

    // 重置内部历史状态（如切换全局路线时调用）
    virtual void reset() {}

    virtual void set_reference_speed(double reference_speed)
    {
        _reference_speed = reference_speed;
    }

    virtual double reference_speed() const
    {
        return _reference_speed;
    }

protected:
    double _reference_speed = 12.0;
};
