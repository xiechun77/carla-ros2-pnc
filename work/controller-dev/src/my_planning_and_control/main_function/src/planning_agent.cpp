#include "planning_agent.h"

class PlanningAgent : public rclcpp::Node
{
public:
  PlanningAgent() : Node("planning_agent")// 基类初始化
  {

    _role_name = "ego_vehicle";// 主车名称
    

    // 订阅方
    _current_ego_state = std::make_shared<VehicleState>();
    // 创建里程计订阅方，订阅车辆当前位姿消息
    _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        // 话题名称
        "/carla/" + _role_name +"/odometry",
        10,
        std::bind(&PlanningAgent::odometry_cb,this,std::placeholders::_1)
    );
    // 创建惯性导航订阅方，订阅车辆当前加速度和角速度消息
    _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "/carla/" + _role_name +"/imu",
      10,
      std::bind(&PlanningAgent::imu_cb,this,std::placeholders::_1)
    );
    // 创建车辆信息订阅方，订阅车辆id号
    _ego_info_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
      "/carla/ego_vehicle/vehicle_info",
      10,
      std::bind(&PlanningAgent::ego_info_cb,this,std::placeholders::_1)
    );
    
    // 创建期望速度订阅方
    _target_speed_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        "/carla/" + _role_name +"/speed_command",
        10,
        std::bind(&PlanningAgent::target_speed_cb,this,std::placeholders::_1)
    );

    // 创建路径订阅方
    _path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
        "/carla/" + _role_name + "/waypoints",
        10,
        std::bind(&PlanningAgent::path_cb,this,std::placeholders::_1)
    );
    _object_array_subscriber = this->create_subscription<derived_object_msgs::msg::ObjectArray>(
      "/carla/ego_vehicle/objects",
      10,
      std::bind(&PlanningAgent::objects_cb,this,std::placeholders::_1)
    );


    // 发布方
    _trajectory_publisher = this->create_publisher<pnc_msgs::msg::Trajectory>(
        "/carla/" + _role_name + "/planning_trajectory",
        10
    );
    // 创建局部路径可视化发布者
    _local_path_publisher = this->create_publisher<nav_msgs::msg::Path>(
        "/carla/" + _role_name + "/local_path",
        10
    );  
    // 创建 SL/ST 调试数据发布者（JSON 字符串，供 planning_debug_plotter.py 绘图/记录）
    _planning_debug_publisher = this->create_publisher<std_msgs::msg::String>(
        "/carla/" + _role_name + "/planning_debug",
        10
    );

    // 规划部分（支持运行时动态切换 planner_type: em | lattice）
    _planner_type = this->declare_parameter<std::string>("planner_type", "em");
    if (_planner_type == "em")
    {
      _planner = std::make_shared<EMPlanner>();
    }
    else if (_planner_type == "lattice")
    {
      _planner = std::make_shared<LatticePlanner>();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "未知 planner_type='%s'（当前支持: em, lattice），回退到 em",
                  _planner_type.c_str());
      _planner_type = "em";
      _planner = std::make_shared<EMPlanner>();
    }
    RCLCPP_INFO(this->get_logger(), "PlanningAgent 已启用规划器: %s", _planner->name().c_str());

    _planning_time_step = 0.1;// 100ms执行一次规划
    _reference_line_generator = std::make_shared<ReferenceLine>();// 参考线生成器
    _planning_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(_planning_time_step*1000)),std::bind(&PlanningAgent::planning_run_step,this));



    // 绘图
    // _reference_line_figure_handle = matplot::figure();
    // _final_path_figure_handle = matplot::figure();
  }

private:
  // 参数

  // 主车名称
  std::string _role_name;

  // 订阅方以及订阅的数据
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;// 里程计订阅方，订阅本车当前位姿与速度
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;// 惯性导航订阅方，订阅加速度与角速度
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr _ego_info_subscriber;// 定于车辆的车道信息
  std::shared_ptr<VehicleState> _current_ego_state; // 用于存储主车信息
  rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr _object_array_subscriber;// 对象序列，包含本车和障碍物
  std::vector<derived_object_msgs::msg::Object> _object_arrry;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _target_speed_subscriber;// 期望速度订阅方
  double _reference_speed;// 期望速度,单位为km/h

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscriber;// 路径订阅方
  std::vector<PathPoint> _global_path;// 存储全局路径

  // 发布方
  rclcpp::Publisher<pnc_msgs::msg::Trajectory>::SharedPtr _trajectory_publisher;
  // 发布用于 RViz 显示的局部规划路径
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _local_path_publisher;
  // 发布 SL/ST 调试数据
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _planning_debug_publisher;

  // 规划环节
  std::string _planner_type;
  double _planning_time_step;
  rclcpp::TimerBase::SharedPtr _planning_timer;
  std::shared_ptr<ReferenceLine> _reference_line_generator;// 参考线生成器
  std::vector<PathPoint> _reference_line;
  std::shared_ptr<PlannerBase> _planner;
  std::vector<TrajectoryPoint> _trajectory;

  // 绘图
  size_t _count_plot = 0;
  matplot::figure_handle _reference_line_figure_handle;
  matplot::figure_handle _final_path_figure_handle;


public:
  // *******************************************************
  // 回调函数
  // 里程计订阅方回调函数，获取当前本车位姿与速度
  void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    _current_ego_state->x = msg->pose.pose.position.x;
    _current_ego_state->y = msg->pose.pose.position.y;
    _current_ego_state->z = msg->pose.pose.position.z;
    _current_ego_state->v = std::sqrt(std::pow(msg->twist.twist.linear.x,2) 
                            +  std::pow(msg->twist.twist.linear.y,2)
                            +  std::pow(msg->twist.twist.linear.z,2));
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg->pose.pose.orientation,tf_q);
    double roll,pitch,yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll,pitch,yaw);
    _current_ego_state->heading = tf2NormalizeAngle(yaw);
    _current_ego_state->flag_ode = true;
    // 里程计的单位是m/s，转化为km/h
  }

  void imu_cb(sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    if (imu_msg->linear_acceleration.x >= 10 || imu_msg->linear_acceleration.y >= 10)
    {
      return;
    }
    
    _current_ego_state->ax = imu_msg->linear_acceleration.x;
    _current_ego_state->ay = imu_msg->linear_acceleration.y;
    _current_ego_state->flag_imu = true;
  }

  void ego_info_cb(carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info)
  {
    _current_ego_state->id = ego_info->id;
    _current_ego_state->flag_info = true;
  }

  // 速度指令订阅方回调函数，获取有ad_agent发布的速度指令
  void target_speed_cb(std_msgs::msg::Float64::SharedPtr msg)
  {
    _reference_speed = msg->data;
  }

  void path_cb(nav_msgs::msg::Path::SharedPtr waypoints)
  {
    //  RCLCPP_INFO(this->get_logger(),"接收到全局路径信息......");
    _global_path.clear();
    _trajectory.clear();
    _reference_line.clear();
    _reference_line_generator = std::make_shared<ReferenceLine>();
    if (_planner)
    {
      _planner->reset();
    }
    for (auto &&pose : waypoints->poses)
    {

      PathPoint temp_path_point;
      temp_path_point.x = pose.pose.position.x;
      temp_path_point.y = pose.pose.position.y;
      _global_path.push_back(temp_path_point);

    }
    if (_global_path.size() >= 3)
    {
      Calculate_heading_and_kappa(_global_path);
    }
  }// 全局路径订阅回调


  void objects_cb(derived_object_msgs::msg::ObjectArray::SharedPtr object_array)
  {
    _object_arrry.clear();
    for (auto &&object : object_array->objects)
    {
      _object_arrry.push_back(object);
    }
    
  }



  void planning_run_step()// 单步规划
  {
    // 1.信息检查
    if(_global_path.size() < 3)// 等待接受到路径
    {
      RCLCPP_ERROR(this->get_logger(),"全局路径信息缺失,等待全局路径信息!!!");
      return;
    }
    // 等待接受到主车信息
    if(_current_ego_state->flag_imu == false ||
       _current_ego_state->flag_ode == false ||
       _current_ego_state->flag_info == false)
    {
      RCLCPP_ERROR(this->get_logger(),"车辆信息缺失,等待车辆信息！！！");
      return;
    }

    // 2.生成参考线
    _reference_line.clear();
    if(_reference_line_generator->run_step(_current_ego_state,_global_path,_reference_line))
    {
      RCLCPP_DEBUG(this->get_logger(),"参考线生成成功！");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),"参考线生成失败！");
    }
    RCLCPP_DEBUG(this->get_logger(),"mian错误定位1");

    // 3.调用当前规划器生成轨迹
    const double current_time = this->now().seconds();
    _planner->plan(current_time, _reference_line, _current_ego_state, _object_arrry, _trajectory);
    RCLCPP_DEBUG(this->get_logger(),"mian错误定位2");

    // 4.发布轨迹
    pnc_msgs::msg::TrajectoryPoint pub_point;
    pnc_msgs::msg::Trajectory pub_trajectory;
    for (auto & point : _trajectory)
    {
      pub_point.x = point.x;
      pub_point.y = point.y;
      pub_point.heading = point.heading;
      pub_point.kappa = point.kappa;
      pub_point.v = point.v;
      pub_point.ax = point.ax;
      pub_point.ay = point.ay;
      pub_point.a_tau = point.a_tau;
      pub_point.time_stamped = point.time_stamped;

      pub_trajectory.points.emplace_back(pub_point);
    }
    _trajectory_publisher->publish(pub_trajectory);
  // 5.发布用于 RViz 显示的局部路径
    nav_msgs::msg::Path local_path;
    local_path.header.frame_id = "map";
    local_path.header.stamp = this->now();

    for (const auto & point : _trajectory)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = local_path.header;

      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = 0.2;

      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, point.heading);
      pose.pose.orientation = tf2::toMsg(quaternion);

      local_path.poses.push_back(pose);
    }

    _local_path_publisher->publish(local_path);

    // 6.发布 SL/ST 调试数据（没有订阅者时跳过序列化）
    if (_planning_debug_publisher->get_subscription_count() > 0)
    {
      std_msgs::msg::String debug_msg;
      debug_msg.data = planning_debug_to_json(_planner->debug_data());
      _planning_debug_publisher->publish(debug_msg);
    }
   // -------------------------绘图---------------------
    // if(_count_plot % 20 == 0)
    // {
    //   matplot::figure(_reference_line_figure_handle);
    //   matplot::cla();
    //   std::vector<double> global_x,global_y,reference_x,reference_y,trajectory_x,trajectory_y;
    //   for (auto &&path_point : _global_path)
    //   {
    //     global_x.push_back(path_point.x);
    //     global_y.push_back(path_point.y);
    //   }

    //   // matplot::hold(false);
    //   matplot::plot(global_x,global_y,"go");
    //   matplot::hold(true);
    //   for (auto &&path_point : _reference_line)
    //   {
    //     reference_x.push_back(path_point.x);
    //     reference_y.push_back(path_point.y);
    //   }
    //   matplot::plot(reference_x,reference_y,"--rx");

    //   int plot_index = 0;
    //   for (auto &&trajectory_point : _trajectory)
    //   {
    //     if (plot_index % 10 == 0)
    //     {
    //       trajectory_x.emplace_back(trajectory_point.x);
    //       trajectory_y.emplace_back(trajectory_point.y);
    //     }
    //     plot_index ++ ;

    //   }
      
    //   matplot::plot(trajectory_x, trajectory_y,"-b*");

    //   matplot::plot({_current_ego_state->x}, {_current_ego_state->y}, "ro");
    //   matplot::hold(false);
    //   // h->draw();
    // }
    // _count_plot++ ;

    RCLCPP_INFO(this->get_logger(),"main错误定位3");
  }
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<PlanningAgent>());

  rclcpp::shutdown();
  
  return 0;
}
