#include "control_agent.h"
#include "terminal_control.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

class ControlAgent : public rclcpp::Node
// 控制代理,用于实现产生控制指令
{
public:
    ControlAgent() : Node("control_agent")
    {
        _role_name = "ego_vehicle";

        // 订阅方
        // 里程计信息订阅方
        _ego_state = std::make_shared<VehicleState>();
        _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/" + _role_name +"/odometry",
            10,
            std::bind(&ControlAgent::odometry_cb,this,std::placeholders::_1) 
        );
        // 创建惯性导航订阅方，订阅车辆当前加速度和角速度消息
        _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "/carla/" + _role_name +"/imu",
            10,
            std::bind(&ControlAgent::imu_cb,this,std::placeholders::_1)
        );
        // 创建车辆信息订阅方，订阅车辆id号
        _ego_info_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
            "/carla/ego_vehicle/vehicle_info",
            10,
            std::bind(&ControlAgent::ego_info_cb,this,std::placeholders::_1)
        );
        _trajectory_subscriber = this->create_subscription<pnc_msgs::msg::Trajectory>(
             "/carla/" + _role_name + "/planning_trajectory",
             10,
             std::bind(&ControlAgent::trajectory_cb,this,std::placeholders::_1)
        );
        _path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
            "/carla/" + _role_name + "/waypoints", 10,
            [this](nav_msgs::msg::Path::SharedPtr msg) {
                _route.clear();
                for (const auto& pose : msg->poses)
                    _route.emplace_back(pose.pose.position.x, pose.pose.position.y);
                _goal_stopped = false;
                _trajectory.clear(); // wait for a trajectory belonging to the new route
            });


        // 发布方
        _control_cmd_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/" + _role_name + "/vehicle_control_cmd",
            10
        );
        _real_speed_publisher = this->create_publisher<std_msgs::msg::Float64>(
            "real_speed",
            10
        );
        _real_heading_publisher = this->create_publisher<std_msgs::msg::Float64>(
            "real_heading",
            10
        );
        _control_debug_publisher = this->create_publisher<std_msgs::msg::String>(
            "/carla/" + _role_name + "/control_debug",
            10
        );

        // 控制器类型参数（运行时可切换，无需重编译；当前支持: mpc, lqr, stanley）
        _controller_type = this->declare_parameter<std::string>("controller_type", "mpc");
        if (_controller_type != "mpc" && _controller_type != "lqr" && _controller_type != "stanley") {
            RCLCPP_WARN(this->get_logger(),
                        "未知 controller_type='%s'（当前支持: mpc, lqr, stanley），回退到 mpc",
                        _controller_type.c_str());
            _controller_type = "mpc";
        }
        RCLCPP_INFO(this->get_logger(), "ControlAgent 已启用控制器: %s", _controller_type.c_str());

        // 控制器
        _control_time_step = 0.02;// 20ms执行一次控制
        _control_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(_control_time_step*1000)),
                                                 std::bind(&ControlAgent::control_run_step,this));
        _lon_cascade_pid_controller = std::make_unique<LonCascadePIDController>();
        _lateral_lqr_controller = std::make_unique<LateralLQRController>();
        _lateral_stanley_controller = std::make_unique<LateralStanleyController>();
        _mpc_controller = std::make_unique<MPCController>();


    }
private:
    std::string _role_name ;// 主车名称

    // 订阅方以及订阅的数据
    // 里程计订阅方，订阅本车当前位姿与速度
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;
    // 惯性导航订阅方，订阅加速度与角速度
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;
    // 定于车辆的车道信息
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr _ego_info_subscriber;
    // 主车信息
    std::shared_ptr<VehicleState> _ego_state; 

    // 规划轨迹订阅方
    rclcpp::Subscription<pnc_msgs::msg::Trajectory>::SharedPtr _trajectory_subscriber;
    // 规划轨迹
    std::vector<TrajectoryPoint> _trajectory;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscriber;
    std::vector<std::pair<double,double>> _route;
    bool _goal_stopped = false;

    // 发布方
    // 控制指令发布方
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _control_cmd_publisher;
    // 实际速度发布方,用于rqt绘图
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _real_speed_publisher;
    // 实际航向角发布方,用于rqt绘图
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _real_heading_publisher;
    // 控制调试与评测数据发布方（JSON 字符串）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _control_debug_publisher;

    // 控制器
    std::string _controller_type;
    // 纵向串级PID控制器
    std::unique_ptr<LonCascadePIDController> _lon_cascade_pid_controller;
    // 横向lqr控制器
    std::unique_ptr<LateralLQRController> _lateral_lqr_controller;
    // 横向stanley控制器
    std::unique_ptr<LateralStanleyController> _lateral_stanley_controller;
    // 横纵向MPC控制器
    std::unique_ptr<MPCController> _mpc_controller;
    // 控制器时钟
    rclcpp::TimerBase::SharedPtr _control_timer;
    // 控制时间步
    double _control_time_step;
    
public:
    void control_run_step()
    {

        // 1.信息检查
        // 1.1车辆状态信息检查
        if (_ego_state->flag_imu == false ||
            _ego_state->flag_info == false ||
            _ego_state->flag_ode == false)
        {
            emergency_stop();
            RCLCPP_ERROR(this->get_logger(),"车辆信息缺失,等待车辆信息！！！");
            return;
        }
        
        // 1.2轨迹信息检查
            
        if (_trajectory.empty())
        {
            emergency_stop();
            RCLCPP_ERROR(this->get_logger(),"规划轨迹信息缺失,等待规划轨迹信息！！！");
            return;
        }

        const auto route_position = terminal_control::locate(_route, _ego_state->x, _ego_state->y);
        if (_goal_stopped || (route_position.remaining <= 2.0 &&
                             route_position.goal_distance < 3.5)) {
            _goal_stopped = true;
            emergency_stop();
            return;
        }


        // 2.求解控制指令
        double cur_t = this->now().seconds();
        ControlCMD cmd;
        const auto solve_start = std::chrono::steady_clock::now();
        bool solve_ok = false;

        if (_controller_type == "lqr") {
            // 2.1 纵向串级 PID 控制
            _lon_cascade_pid_controller->set_station_controller(0.1, 0.0, 0.0);
            _lon_cascade_pid_controller->set_speed_controller(2.0, 0.2, 0.0);
            _lon_cascade_pid_controller->set_speed_integral_saturation_boundary(0.3, -0.3);
            const bool lon_ok = _lon_cascade_pid_controller->compute_control_cmd(
                _trajectory, _ego_state, cur_t, _control_time_step, cmd);
            // 2.2 横向 LQR 控制
            _lateral_lqr_controller->set_r_matrix(1.0);
            std::vector<double> q_vector = {0.05, 0.0, 1.0, 0.0};
            _lateral_lqr_controller->set_q_matrix(q_vector);
            const bool lat_ok = _lateral_lqr_controller->compute_control_cmd(
                _trajectory, _ego_state, cur_t, _control_time_step, cmd);
            solve_ok = lon_ok && lat_ok;
        } else if (_controller_type == "stanley") {
            // 2.1 纵向串级 PID 控制（与 LQR 共用相同的纵向控制器，保证横向公平对比）
            _lon_cascade_pid_controller->set_station_controller(0.1, 0.0, 0.0);
            _lon_cascade_pid_controller->set_speed_controller(2.0, 0.2, 0.0);
            _lon_cascade_pid_controller->set_speed_integral_saturation_boundary(0.3, -0.3);
            const bool lon_ok = _lon_cascade_pid_controller->compute_control_cmd(
                _trajectory, _ego_state, cur_t, _control_time_step, cmd);
            // 2.2 横向 Stanley 几何控制
            const bool lat_ok = _lateral_stanley_controller->compute_control_cmd(
                _trajectory, _ego_state, cur_t, _control_time_step, cmd);
            solve_ok = lon_ok && lat_ok;
        } else {
            // 横纵向 MPC 控制
            std::vector<double> q_vector = {50.0, 0.0, 220.0, 0.0, 70.0, 20.0};
            std::vector<double> r_vector = {70.0, 10.0};
            _mpc_controller->set_matrix_Q(q_vector);
            _mpc_controller->set_matrix_R(r_vector);
            solve_ok = _mpc_controller->compute_control_cmd(
                _trajectory, _ego_state, cur_t, _control_time_step, cmd);
        }

        const double solve_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - solve_start).count();

        if (!solve_ok ||
            !std::isfinite(cmd.throttle) || !std::isfinite(cmd.brake) || !std::isfinite(cmd.steer)) {
            emergency_stop();
            return;
        }
        cmd.steer = terminal_control::bound(cmd.steer, -1.0, 1.0);
        cmd.throttle = terminal_control::bound(cmd.throttle, 0.0, 1.0);
        cmd.brake = terminal_control::bound(cmd.brake, 0.0, 1.0);
        // 终点制动区速度包络控制，消除 MPC 恒定油门偏置带来的终点冲墙
        const double stopping_zone = std::max(15.0, _ego_state->v * _ego_state->v / 3.0 + 8.0);
        if (route_position.remaining < stopping_zone && route_position.lateral < 3.5) {
            const double acceleration = 1.5 * (terminal_control::target_speed(route_position.remaining) - _ego_state->v);
            if (acceleration <= 0.0) {
                cmd.throttle = 0.0;
                cmd.brake = std::max(cmd.brake, terminal_control::bound(-acceleration / 6.0, 0.0, 1.0));
            } else {
                cmd.throttle = std::min(cmd.throttle, terminal_control::bound(acceleration / 3.0, 0.0, 1.0));
            }
        }

        // 3.发布控制指令
        carla_msgs::msg::CarlaEgoVehicleControl cmd_msg;
        cmd_msg.brake = cmd.brake;
        cmd_msg.steer = cmd.steer;
        cmd_msg.throttle = cmd.throttle;
        cmd_msg.hand_brake = false;
        cmd_msg.manual_gear_shift = false;
        _control_cmd_publisher->publish(cmd_msg);

        RCLCPP_INFO(this->get_logger(), "[%s] 油门:%.3f, 刹车:%.3f, 转角:%.3f",
                    _controller_type.c_str(), cmd.throttle, cmd.brake, cmd.steer);

        // 发布消息,用于rqt绘图
        std_msgs::msg::Float64 real_speed_msg, real_heading_msg;
        real_speed_msg.data = _ego_state->v;
        real_heading_msg.data = _ego_state->heading;
        _real_speed_publisher->publish(real_speed_msg);
        _real_heading_publisher->publish(real_heading_msg);

        // 4.发布控制调试数据（供统一评测记录器采样，无订阅者时跳过序列化）
        if (_control_debug_publisher->get_subscription_count() > 0) {
            publish_control_debug(cur_t, cmd, solve_time_ms, route_position.remaining);
        }

    }

private:

    void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _ego_state->x = msg->pose.pose.position.x;
        _ego_state->y = msg->pose.pose.position.y;
        _ego_state->z = msg->pose.pose.position.z;
        _ego_state->v = std::sqrt(std::pow(msg->twist.twist.linear.x,2) 
                                +  std::pow(msg->twist.twist.linear.y,2)
                                +  std::pow(msg->twist.twist.linear.z,2));
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg->pose.pose.orientation,tf_q);
        double roll,pitch,yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll,pitch,yaw);
        _ego_state->heading = tf2NormalizeAngle(yaw);
        _ego_state->flag_ode = true;
        // 里程计的单位是m/s，转化为km/h
    }

    void imu_cb(sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {

        _ego_state->ax = imu_msg->linear_acceleration.x;
        _ego_state->ay = imu_msg->linear_acceleration.y;
        _ego_state->flag_imu = true;
    }

    void ego_info_cb(carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info)
    {
        _ego_state->id = ego_info->id;
        _ego_state->flag_info = true;
    }

    void trajectory_cb(pnc_msgs::msg::Trajectory::SharedPtr trajectory)
    {
        _trajectory.clear();
        TrajectoryPoint sub_point; 
        for (auto &point : trajectory->points)
        {
            sub_point.x = point.x;
            sub_point.y = point.y;
            sub_point.heading = point.heading;
            sub_point.kappa = point.kappa;
            sub_point.v = point.v;
            sub_point.ax = point.ax;
            sub_point.ay = point.ay;
            sub_point.a_tau = point.a_tau;
            sub_point.time_stamped = point.time_stamped;

            _trajectory.emplace_back(sub_point);
        }
    }

    // 发布控制跟踪误差与耗时 JSON，供统一评测脚本采集
    void publish_control_debug(double cur_t, const ControlCMD& cmd, double solve_time_ms, double route_remaining)
    {
        double lateral_error = 0.0;
        double heading_error = 0.0;
        double speed_error = 0.0;
        double target_v = 0.0;
        double target_kappa = 0.0;

        if (!_trajectory.empty()) {
            size_t nearest_idx = 0;
            double min_dist_sq = std::numeric_limits<double>::max();
            for (size_t i = 0; i < _trajectory.size(); ++i) {
                const double dx = _ego_state->x - _trajectory[i].x;
                const double dy = _ego_state->y - _trajectory[i].y;
                const double d2 = dx * dx + dy * dy;
                if (d2 < min_dist_sq) {
                    min_dist_sq = d2;
                    nearest_idx = i;
                }
            }
            const auto& ref = _trajectory[nearest_idx];
            const double dx = _ego_state->x - ref.x;
            const double dy = _ego_state->y - ref.y;
            lateral_error = -dx * std::sin(ref.heading) + dy * std::cos(ref.heading);
            heading_error = tf2NormalizeAngle(_ego_state->heading - ref.heading);
            target_v = ref.v;
            target_kappa = ref.kappa;
            speed_error = _ego_state->v - ref.v;
        }

        auto safe_num = [](double v) -> std::string {
            if (!std::isfinite(v)) {
                return "null";
            }
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6) << v;
            return oss.str();
        };

        std::ostringstream js;
        js << "{"
           << "\"t\":" << safe_num(cur_t)
           << ",\"controller_type\":\"" << _controller_type << "\""
           << ",\"lateral_error\":" << safe_num(lateral_error)
           << ",\"heading_error\":" << safe_num(heading_error)
           << ",\"speed_error\":" << safe_num(speed_error)
           << ",\"target_v\":" << safe_num(target_v)
           << ",\"target_kappa\":" << safe_num(target_kappa)
           << ",\"ego_v\":" << safe_num(_ego_state->v)
           << ",\"ego_ax\":" << safe_num(_ego_state->ax)
           << ",\"ego_ay\":" << safe_num(_ego_state->ay)
           << ",\"throttle\":" << safe_num(cmd.throttle)
           << ",\"brake\":" << safe_num(cmd.brake)
           << ",\"steer\":" << safe_num(cmd.steer)
           << ",\"solve_time_ms\":" << safe_num(solve_time_ms)
           << ",\"route_remaining\":" << safe_num(route_remaining)
           << ",\"goal_stopped\":" << (_goal_stopped ? "true" : "false")
           << "}";

        std_msgs::msg::String msg;
        msg.data = js.str();
        _control_debug_publisher->publish(msg);
    }

    // 紧急停车指令
    void emergency_stop()
    {
        carla_msgs::msg::CarlaEgoVehicleControl control_msg;
        control_msg.brake = 1.0;
        control_msg.throttle = 0.0;
        control_msg.steer = 0.0;
        control_msg.reverse = false;
        _control_cmd_publisher->publish(control_msg);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControlAgent>());

    rclcpp::shutdown();

    return 0;
}
