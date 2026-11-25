#pragma once

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "common.h"

/**
 * @brief 横向 Stanley 几何跟踪控制器（前轴中心参考 + 切向连续投影 + 阿克曼曲率前馈）
 *
 * 与 LateralLQRController（线性最优）、MPCController（约束滚动优化）共同构成
 * 控制模块的三大经典流派对比。
 */
class LateralStanleyController
{
public:
    LateralStanleyController();

    bool compute_control_cmd(
        const std::vector<TrajectoryPoint>& trajectory,
        const std::shared_ptr<VehicleState>& ego_state,
        double cur_t,
        double dt,
        ControlCMD& cmd);

    void set_params(double k_y, double k_soft, double k_ff = 0.85, double steer_filter_alpha = 0.45)
    {
        _k_y = k_y;
        _k_soft = k_soft;
        _k_ff = k_ff;
        _steer_filter_alpha = steer_filter_alpha;
    }

private:
    double _wheelbase = 2.852;           // 轴距 L = a + b (m)
    double _lf = 2.852 / 2.0;            // 质心到前轴距离 (m)
    double _k_y = 2.0;                   // 横向误差增益（增强借道 S 弯横向收敛刚度，防止向外漂移压路沿）
    double _k_soft = 1.2;                // 低速软化常数 (m/s)，抑制低速过度转向
    double _k_ff = 0.45;                 // 适度曲率前馈（前轴参考已自带半轴距几何预瞄，避免急弯内切上路沿）
    double _steer_filter_alpha = 0.45;   // 一阶低通滤波系数，消除离散轨迹点切换毛刺
    double _max_steer_rad = 60.0 * M_PI / 180.0;

    bool _has_prev_steer = false;
    double _prev_steer = 0.0;
};
