#include "emplanner.h"
#include <chrono>

auto LOG = rclcpp::get_logger("emplanner");

EMPlanner::EMPlanner()
{
    // 路径决策代价初始化
    _weight_coefficients.path_dp_w_ref = 200.0;
    _weight_coefficients.path_dp_w_dl = 300.0;
    _weight_coefficients.path_dp_w_ddl = 200.0;
    _weight_coefficients.path_dp_w_dddl = 1000.0;
    _weight_coefficients.path_dp_w_obs = 1e5;

    // 路径规划代价初始化
    _weight_coefficients.path_qp_w_ref = 100.0;
    _weight_coefficients.path_qp_w_dl = 500.0;
    _weight_coefficients.path_qp_w_ddl = 50.0;
    _weight_coefficients.path_qp_w_dddl = 10.0;
    _weight_coefficients.path_qp_w_mid = 20.0;
    _weight_coefficients.path_qp_w_l_end = 40.0;
    _weight_coefficients.path_qp_w_dl_end = 40.0;
    _weight_coefficients.path_qp_w_ddl_end = 40.0;

    // 速度决策代价初始化
    _weight_coefficients.speed_dp_w_ref_speed = 1000.0;
    _weight_coefficients.speed_dp_w_a = 300.0;
    _weight_coefficients.speed_dp_w_jerk = 300.0;
    _weight_coefficients.speed_dp_w_obs = 1e7;

    // 速度规划代价初始化
    _weight_coefficients.speed_qp_w_ref_speed = 200.0;
    _weight_coefficients.speed_qp_w_a = 100.0;
    _weight_coefficients.speed_qp_w_jerk = 100.0;

    // 绘图（已改由 planning_debug_plotter.py 独立绘制，禁用此处的 matplot::figure() 以避免每次规划新建后台 gnuplot 进程拖慢 100ms 周期）
    // _path_polt_handle = matplot::figure();
    // _trajectory_plot_handle = matplot::figure();
    _plot_count = 0;

    // 路径二次规划
    _path_qp_solver.settings()->setWarmStart(true);
    _speed_qp_solver.settings()->setWarmStart(true);

    // _reference_speed = 8.0;
    _reference_speed = 12.0;

}


//  void EMPlanner::reference_speed_callback(std_msgs::msg::Float64::SharedPtr msg)
//  {
//      //   _reference_speed = msg->data/3.6; // km/h转化为m/s
//      //   RCLCPP_INFO(LOG,"参考速度为:",_reference_speed);
//  }

void EMPlanner::plan(
    double current_time,
    const std::vector<PathPoint>& reference_line,
    const std::shared_ptr<VehicleState>& ego_state,
    const std::vector<derived_object_msgs::msg::Object>& obstacles,
    std::vector<TrajectoryPoint>& final_trajectory)
{
    _current_time = current_time;
    // 调试数据：每周期重新填充，只记录不参与计算
    const auto debug_start_time = std::chrono::steady_clock::now();
    _debug_data.clear();
    _debug_data.timestamp = _current_time;
    _debug_data.reference_speed = _reference_speed;
    _debug_data.ego_v = ego_state->v;
    // 1障碍物处理
    // 1.1区别动态与静态障碍物，存储在类成员变量里
    obstacle_fileter(ego_state, obstacles);
    // 2.2障碍物坐标转换
    auto reference_index2s = calculate_index_to_s(reference_line,ego_state);
    //  for (auto &&s : reference_index2s)
    //  {
    //      RCLCPP_INFO(LOG,"index2s表%.3f",s);
    //  }
    
    std::vector<FrenetPoint> static_obstacle_frenet_coordinate;

    if(!_static_obstacles.empty())
    {

        cartesion_set_to_frenet_set(_static_obstacles, reference_line, ego_state, static_obstacle_frenet_coordinate);

        for (auto && s_obs_frenet : static_obstacle_frenet_coordinate)
        {
            RCLCPP_INFO(LOG,"静态障碍物信息(%.3f,%.3f)",s_obs_frenet.s,s_obs_frenet.l);
        }
        
        
    }
    // 调试：静态障碍物、动态障碍物（仅显示）与主车在参考线上的 SL 坐标
    std::vector<FrenetPoint> dynamic_obs_ref_sl;
    if (!_dynamic_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_dynamic_obstacles, reference_line, ego_state, dynamic_obs_ref_sl);
    }
    _debug_data.dynamic_obs_sl = dynamic_obs_ref_sl;

    // 1.3 同车道慢速前车借道超车决策（Apollo borrow-lane nudge 思路）
    // 检查左侧对向车道 [-10m, 55m] 内是否有来车，确保借道安全
    bool left_lane_clear = true;
    for (const auto& d_sl : dynamic_obs_ref_sl)
    {
        if (d_sl.l > 1.5 && d_sl.l < 6.0 && d_sl.s > -10.0 && d_sl.s < 55.0 && d_sl.s_dot < -0.5)
        {
            left_lane_clear = false;
            break;
        }
    }
    for (const auto& s_sl : static_obstacle_frenet_coordinate)
    {
        if (s_sl.l > 1.5 && s_sl.l < 5.5 && s_sl.s > -5.0 && s_sl.s < 45.0)
        {
            left_lane_clear = false;
            break;
        }
    }

    std::unordered_set<uint32_t> active_overtake_ids;
    for (size_t i = 0; i < _dynamic_obstacles.size() && i < dynamic_obs_ref_sl.size(); ++i)
    {
        const uint32_t obs_id = _dynamic_obstacles[i].id;
        const FrenetPoint& d_sl = dynamic_obs_ref_sl[i];
        const bool was_overtaking = (_overtake_lead_ids.count(obs_id) > 0);

        // 同车道同向慢车判定：
        // - 首次触发：前方 (-2m < s < 38m)，处于本车道 (|l| < 1.8m)，同向慢速 (0 <= s_dot < 0.75 * ref_speed)，且左道空闲
        // - 状态保持（Latch）：一旦开始超车，直到完全超越该车（s < -6m）才释放，防止超车中途因接近或并排而切回原车道发生擦碰
        bool should_overtake = false;
        if (was_overtaking)
        {
            if (d_sl.s > -6.0 && d_sl.s < 50.0 && std::abs(d_sl.l) < 2.5 && d_sl.s_dot < _reference_speed * 0.9)
            {
                should_overtake = true;
            }
        }
        else if (left_lane_clear)
        {
            if (d_sl.s > -2.0 && d_sl.s < 38.0 && std::abs(d_sl.l) < 1.8 &&
                d_sl.s_dot >= -0.2 && d_sl.s_dot < _reference_speed * 0.75 && std::abs(d_sl.l_dot) < 0.8)
            {
                should_overtake = true;
            }
        }

        if (should_overtake)
        {
            active_overtake_ids.insert(obs_id);
            // 将该慢车在当前至未来短时内占据的纵向走廊转化为 SL 虚拟静态障碍物点列，供路径 DP 和凸空间左侧绕行
            const double v_ego_est = std::max({ego_state->v, _reference_speed * 0.8, 4.0});
            const double rel_v = std::max(1.5, v_ego_est - std::max(0.0, d_sl.s_dot));
            const double t_pass = clamp((std::max(0.0, d_sl.s) + 6.0) / rel_v, 0.0, 3.5);
            const double lead_advance = clamp(std::max(0.0, d_sl.s_dot) * t_pass, 3.0, 14.0);

            for (double ds = -2.0; ds <= lead_advance; ds += 3.5)
            {
                FrenetPoint virt_obs = d_sl;
                virt_obs.s = d_sl.s + ds;
                static_obstacle_frenet_coordinate.push_back(virt_obs);
            }
            RCLCPP_INFO(LOG, "借道超车目标 id=%u, s=%.2f, l=%.2f, v=%.2f, 纵向扩展=[%.1f, %.1f]",
                        obs_id, d_sl.s, d_sl.l, d_sl.s_dot, d_sl.s - 2.0, d_sl.s + lead_advance);
        }
    }
    _overtake_lead_ids = std::move(active_overtake_ids);
    _debug_data.static_obs_sl = static_obstacle_frenet_coordinate;

    {
        std::vector<FrenetPoint> ego_sl_set;
        cartesion_set_to_frenet_set(vehicle_state_to_trajectory_point(ego_state), reference_line, ego_state, ego_sl_set);
        if (!ego_sl_set.empty()) { _debug_data.ego_sl = ego_sl_set.front(); }
    }

    // 2.确定规划起点并投影到frenet坐标系
    TrajectoryPoint planning_start_point = _stitcher.compute_planning_start_point(_current_time, 0.1, ego_state);
    std::vector<FrenetPoint> temp_set;
    cartesion_set_to_frenet_set(planning_start_point, reference_line, ego_state, temp_set);
    FrenetPoint planning_start_point_frenet_coordinate = temp_set.front();
    planning_start_point_frenet_coordinate.l_prime_prime = 0.0;
    _debug_data.start_sl = planning_start_point_frenet_coordinate;


    // -----------------------------------3.路径规划--------------------------------------
    // 3.1获取车道宽度
    double lane_width = 4.0;
    // double car_width = 2.0;

    // 这部分注释是调用服务通信获取路宽,因为路宽不变这里直接赋值了

    //  auto get_waypoint_request = std::make_shared<carla_waypoint_types::srv::GetWaypoint::Request>();
    //  get_waypoint_request->location.x = ego_state->x;
    //  get_waypoint_request->location.y = ego_state->y;
    //  get_waypoint_request->location.z = ego_state->z;
    //  auto response = _get_waypoint_client->async_send_request(get_waypoint_request);
    //  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),response) == rclcpp::FutureReturnCode::SUCCESS)
    //  {
    //      lane_width = response.get()->waypoint.lane_width;

    //          RCLCPP_INFO(LOG,"请求成功,车道宽度为:%.3f",lane_width);
    //  }

    // 3.2dp路径规划
    // 3.2.1获取采用点
    std::vector<std::vector<FrenetPoint>> path_dp_sample;
    get_path_dp_sample(path_dp_sample,planning_start_point_frenet_coordinate,10.0,6,1.0,7);
    // 调试：DP 采样网格（第 0 列是规划起点，不记录）
    for (size_t level = 1; level < path_dp_sample.size(); level++)
    {
        if (!path_dp_sample[level].empty()) { _debug_data.dp_sample_s.push_back(path_dp_sample[level].front().s); }
    }
    if (path_dp_sample.size() > 1)
    {
        for (auto &&point : path_dp_sample[1]) { _debug_data.dp_sample_l.push_back(point.l); }
    }

    //  for (size_t level = 1; level <= path_dp_sample.size() ; level++)
    //  {
    //      RCLCPP_INFO(LOG,"-------第%d列采样点--------",level);
    //      auto vector_point = path_dp_sample[level-1];
    //      for (auto &&point : vector_point)
    //      {
    //          RCLCPP_INFO(LOG,"采样点坐标(%.3f,%.3f)",point.s,point.l);
    //      }   
    //  }

    // 3.2.2获取最小代价
    std::vector<std::vector<DpPathNode>> path_dp_node_table;
    path_dp_node_table.emplace_back();
    path_dp_node_table.back().emplace_back(planning_start_point_frenet_coordinate,0.0,nullptr);
    auto path_dp_sample_front = path_dp_sample.front().front();
    for (size_t level = 1; level < path_dp_sample.size(); level++)
    {
        path_dp_node_table.emplace_back();
        if (level == 1)// 第一列
        {   
            // 遍历第一列每一个节点，与起始点计算代价，算出来的代价直接是最小代价
            for (auto && current_sample_point : path_dp_sample[level])
            {
                path_dp_node_table.back().emplace_back(
                    current_sample_point,
                    calculate_dp_cost(path_dp_sample_front, current_sample_point, static_obstacle_frenet_coordinate, _weight_coefficients),
                    std::make_shared<DpPathNode>(path_dp_node_table.front().front()));
            }
        }
        else
        {
            // 对于不是第一列的点，要对该列的每一个点与前一列的每一个计算代价
            for (auto && current_sample_point : path_dp_sample[level])
            {
                double min_cost = std::numeric_limits<double>::max();
                size_t index;
                for (size_t i = 0 ; i < path_dp_node_table[level -1].size(); i++)
                {
                    // 当前代价 = 前一列点的最小代价 + 前一列点到当前点的代价
                    double cur_cost =
                    path_dp_node_table[level-1][i].min_cost +
                    calculate_dp_cost(path_dp_node_table[level-1][i].sl_point, current_sample_point, static_obstacle_frenet_coordinate, _weight_coefficients);
                    if ( cur_cost < min_cost )
                    {
                        // 记录最小代价与相应的前一列点的索引
                        min_cost = cur_cost;
                        index = i;
                    }              
                }
                path_dp_node_table.back().emplace_back(
                    current_sample_point,
                    min_cost,
                    std::make_shared<DpPathNode>(path_dp_node_table[level-1][index])
                );
            }
        }
    }

    // 3.2.3利用path_dp_node_table找出最优路径
    size_t path_min_cost_index;
    double path_min_cost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_dp_node_table.back().size(); i++)
    {
        if (path_dp_node_table.back()[i].min_cost < path_min_cost)
        {
            path_min_cost = path_dp_node_table.back()[i].min_cost;
            path_min_cost_index = i;
        }
    }
    auto path_useful_node = path_dp_node_table.back()[path_min_cost_index];// 最后一列的最小代价节点
    
    std::vector<FrenetPoint> dp_best_path;// 存放dp产生的最优路径
    dp_best_path.emplace_back(path_useful_node.sl_point);
    while (true)
    {
        // 由最后一列的最小代价节点中的指针，一步一步向前推，把整条路径找出来给
        path_useful_node = *(path_useful_node.pre_node);
        dp_best_path.emplace(dp_best_path.begin(),path_useful_node.sl_point);
        if(dp_best_path.front().s == planning_start_point_frenet_coordinate.s &&
           dp_best_path.front().l == planning_start_point_frenet_coordinate.l)
        {
            break;
        }
    }
    RCLCPP_INFO(LOG,"emplanner错误定位1");
    // 3.2.4对动态规划出来的路径进行加密
    std::vector<FrenetPoint> final_dp_path;
    increased_dp_path(dp_best_path, 1.0, final_dp_path);

    // 3.2.5获取凸空间
    // 现在假定静态障碍物都是车，一般化情况应该是根据相应的障碍物类型获得相应的边框
    std::vector<double> final_dp_path_l_max;
    std::vector<double> final_dp_path_l_min;
    // double safe_distance = 0.0;
    double safe_distance = 0.5;
    double road_up_boundary = 1.5 * lane_width  - safe_distance;
    double road_low_boundary = -(0.5 * lane_width  - safe_distance);
    generate_convex_space(final_dp_path, road_up_boundary, road_low_boundary, static_obstacle_frenet_coordinate, final_dp_path_l_max, final_dp_path_l_min);
    _debug_data.dp_path = dp_best_path;
    _debug_data.dense_dp_path = final_dp_path;
    _debug_data.path_l_min = final_dp_path_l_min;
    _debug_data.path_l_max = final_dp_path_l_max;
    _debug_data.road_up_boundary = road_up_boundary;
    _debug_data.road_low_boundary = road_low_boundary;

    RCLCPP_INFO(LOG,"emplanner错误定位2");
    // 3.3qp路径规划
    std::vector<FrenetPoint> init_qp_path;
    double l_desire = 0.0;
    double dl_desire = 0.0;
    double ddl_desire = 0.0;
    _debug_data.path_qp_ok = path_QP_planning(final_dp_path, final_dp_path_l_min, final_dp_path_l_max, l_desire, dl_desire, ddl_desire, _weight_coefficients, init_qp_path);
    if (!_debug_data.path_qp_ok || init_qp_path.empty())
    {
        RCLCPP_WARN(LOG, "路径 QP 求解失败，回退使用加密 DP 路径");
        init_qp_path = final_dp_path;
    }
    _debug_data.qp_path = init_qp_path;

    // 将路径转化为轨迹
    auto dp_trajectory = frenet_to_cartesion(init_qp_path, reference_line, reference_index2s);


    // -----------------------------------4.速度规划--------------------------------------
    
    // 4.1路径规划结果的index_to_s表
    std::vector<double> path_index2s;
    path_index2s.emplace_back(0.0);
    for (size_t i = 1; i < dp_trajectory.size(); i++)
    {
        path_index2s.emplace_back(path_index2s.back() + std::hypot(dp_trajectory[i].x - dp_trajectory[i-1].x,
                                                                  dp_trajectory[i].y - dp_trajectory[i-1].y));
    }

    // 4.2动态障碍物投影
    std::vector<FrenetPoint> dynamic_obstacle_frenet_coordinate;
    if(!_dynamic_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_dynamic_obstacles, dp_trajectory, dp_trajectory.front(),dynamic_obstacle_frenet_coordinate);
    }
    // ---------------测试
    // 为了验证速度规划，手动添加几个动态障碍物
    //  FrenetPoint test1, test2;
    //  test1.s = 20;
    //  test1.l = 4;
    //  test1.s_dot = 1;
    //  test1.l_dot = -1;
    //  test2.s = 8;
    //  test2.l = 4;
    //  test2.s_dot = 1;
    //  test2.l_dot = -2;
    //  dynamic_obstacle_frenet_coordinate.emplace_back(test1);
    //  dynamic_obstacle_frenet_coordinate.emplace_back(test2);
    //  _reference_speed = 8;
    // 4.3生成st图
    std::vector<std::unordered_map<std::string, double>> dynamic_obs_st_graph;
    if (!dynamic_obstacle_frenet_coordinate.empty())
    {
        generate_st_graph(dynamic_obstacle_frenet_coordinate, 2.0, dynamic_obs_st_graph); 
    }
    _debug_data.path_length = path_index2s.empty() ? 0.0 : path_index2s.back();
    _debug_data.dynamic_obs_on_path = dynamic_obstacle_frenet_coordinate;
    _debug_data.st_graph = dynamic_obs_st_graph;

    // 4.4计算规划起点信息与终点减速包络
    STPoint planning_start_point_st_coordinate = calculate_speed_start_point(planning_start_point);
    const double route_remaining_s = reference_index2s.empty() ? 0.0 : reference_index2s.back();
    const double stop_buffer_m = 2.0; // 在全局路径终点前 2.0m 停稳
    const double terminal_stop_s = std::max(0.0, route_remaining_s - planning_start_point_frenet_coordinate.s - stop_buffer_m);
    // 接近全局路径终点时（剩余弧长 < 50m），按 1.5 m/s^2 舒适减速度平滑降低有效参考速度
    double effective_ref_speed = _reference_speed;
    if (route_remaining_s < 50.0)
    {
        double terminal_v_env = std::sqrt(2.0 * 1.5 * terminal_stop_s);
        effective_ref_speed = std::min(effective_ref_speed, terminal_v_env);
    }
    _debug_data.reference_speed = effective_ref_speed;

    // 4.5速度动态规划
    // 4.5.1获取采样点
    std::vector<std::vector<STPoint>> speed_dp_sample;
    get_speed_dp_sample(planning_start_point_st_coordinate, 0.5, 8.0, path_index2s, speed_dp_sample);
    // 调试：速度 DP 采样网格（每列 s 采样相同）
    for (size_t level = 1; level < speed_dp_sample.size(); level++)
    {
        if (!speed_dp_sample[level].empty()) { _debug_data.st_sample_t.push_back(speed_dp_sample[level].front().t); }
    }
    if (speed_dp_sample.size() > 1)
    {
        for (auto &&point : speed_dp_sample[1]) { _debug_data.st_sample_s.push_back(point.s); }
    }
    // 4.5.2计算代价
    std::vector<std::vector<DpSpeedNode>> speed_dp_node_table;
    speed_dp_node_table.emplace_back();
    auto speed_dp_sample_front = speed_dp_sample.front().front();
    speed_dp_node_table.back().emplace_back(speed_dp_sample_front,0.0,nullptr);
    for (size_t level = 1; level < speed_dp_sample.size(); level++)
    {
        if (level == 1)
        {
            speed_dp_node_table.emplace_back();
            for (auto &&sample_point : speed_dp_sample[level])
            {
                STPoint pre_st_point(speed_dp_sample_front);
                STPoint cur_st_point(sample_point);
                double delat_t = cur_st_point.t - pre_st_point.t;
                cur_st_point.s_dot = (cur_st_point.s - pre_st_point.s)/delat_t;
                cur_st_point.s_dot_dot = (cur_st_point.s_dot - pre_st_point.s_dot)/delat_t;

                speed_dp_node_table.back().emplace_back(
                    cur_st_point,
                    calculate_speed_dp_cost(speed_dp_sample_front, sample_point, effective_ref_speed, dynamic_obs_st_graph, _weight_coefficients),
                    std::make_shared<DpSpeedNode>(speed_dp_node_table.front().front())
                );
            }
        }
        else
        {
            speed_dp_node_table.emplace_back();
            for (int i = 0; i < static_cast<int>(speed_dp_sample[level].size()); i++)
            {
                double min_speed_dp_cost = std::numeric_limits<double>::max();
                int pre_index = -1;
                for (int j = 0; j < (int)speed_dp_sample[level-1].size(); j++)
                {
                    double cur_cost = calculate_speed_dp_cost(speed_dp_node_table[level-1][j].st_point, speed_dp_sample[level][i],effective_ref_speed,dynamic_obs_st_graph,_weight_coefficients)
                                    + speed_dp_node_table[level-1][j].min_cost; // 注意，这里面的start_point不能用speed_dp_sample里的点，因为那里面的没有赋值
                    if(cur_cost < min_speed_dp_cost)
                    {
                        min_speed_dp_cost = cur_cost;
                        pre_index = j;
                    }  
                }
                STPoint pre_st_point(speed_dp_node_table[level-1][pre_index].st_point);
                STPoint cur_st_point(speed_dp_sample[level][i]);
                double delat_t = cur_st_point.t - pre_st_point.t;
                cur_st_point.s_dot = (cur_st_point.s - pre_st_point.s)/delat_t;
                cur_st_point.s_dot_dot = (cur_st_point.s_dot - pre_st_point.s_dot)/delat_t;

                speed_dp_node_table.back().emplace_back(
                    cur_st_point,
                    min_speed_dp_cost,
                    std::make_shared<DpSpeedNode>(speed_dp_node_table[level-1][pre_index])
                );
            }
        }    
    }
    // 4.5.3回溯出轨迹
    double trajectory_min_cost = std::numeric_limits<double>::max();
    int trajectory_min_cost_level = -1;
    int trajectory_min_cost_index = -1;
    for (int level = 1; level < (int)speed_dp_node_table.size(); level++)
    {   // 搜索上边界
        if (speed_dp_node_table[level].front().min_cost < trajectory_min_cost)
        {
            trajectory_min_cost = speed_dp_node_table[level].front().min_cost;
            trajectory_min_cost_level = level;
            trajectory_min_cost_index = 0;   
        }
    }
    for (int index = 0; index < (int)speed_dp_node_table.back().size(); index++)
    {   // 搜索右边界 
        if (speed_dp_node_table.back()[index].min_cost < trajectory_min_cost)
        {
            trajectory_min_cost = speed_dp_node_table.back()[index].min_cost;
            trajectory_min_cost_level = (int)speed_dp_node_table.size()-1;
            trajectory_min_cost_index = index;
        }
    }
    DpSpeedNode trajectory_useful_node(speed_dp_node_table[trajectory_min_cost_level][trajectory_min_cost_index]);
    std::deque<STPoint> dp_speed_profile;
    dp_speed_profile.emplace_front(trajectory_useful_node.st_point);
    while (true)
    {
        trajectory_useful_node = *trajectory_useful_node.pre_node;
        dp_speed_profile.emplace_front(trajectory_useful_node.st_point);
        if (trajectory_useful_node.st_point == speed_dp_sample_front)
        {
            break;
        }
    }
    RCLCPP_INFO(LOG,"emplanner错误定位3");
    // 4.6速度二次规划
    // 4.6.1生成凸空间
    std::vector<double> s_lb, s_ub, s_dot_lb, s_dot_ub;
    generate_convex_space(dp_trajectory, path_index2s, dp_speed_profile, dynamic_obs_st_graph, 0.4*9.8, s_lb, s_ub, s_dot_lb, s_dot_ub);
    // 终点停车墙约束（Apollo Destination Stop Fence）：当终点进入当前规划路径范围时，将 ST 上界 s_ub 封顶在终点前
    if (route_remaining_s < 65.0)
    {
        for (size_t i = 1; i < s_ub.size(); ++i)
        {
            s_ub[i] = std::min(s_ub[i], std::max(s_lb[i] + 0.05, terminal_stop_s));
        }
    }
    _debug_data.dp_speed_profile.assign(dp_speed_profile.begin(), dp_speed_profile.end());
    _debug_data.s_lb = s_lb;
    _debug_data.s_ub = s_ub;
    _debug_data.s_dot_lb = s_dot_lb;
    _debug_data.s_dot_ub = s_dot_ub;
    
    // 4.6.2二次规划
    std::vector<STPoint> init_qp_speed_profile;
    _debug_data.speed_qp_ok = speed_QP_planning(dp_speed_profile, s_lb, s_ub, effective_ref_speed, s_dot_lb, s_dot_ub, _weight_coefficients, init_qp_speed_profile);
    if (!_debug_data.speed_qp_ok || init_qp_speed_profile.empty())
    {
        RCLCPP_WARN(LOG, "速度 QP 求解失败，回退使用 DP 速度剖面");
        init_qp_speed_profile.assign(dp_speed_profile.begin(), dp_speed_profile.end());
    }
    _debug_data.qp_speed_profile = init_qp_speed_profile;

    // 4.6.3加密二次规划得到的速度曲线
    std::vector<STPoint> final_qp_speed_profile;
    increased_speed_profile(init_qp_speed_profile, final_qp_speed_profile);

    // -----------------------------生成最终的轨迹----------------------------------

    // 5.1将路径规划结果与速度规划结构拼成轨迹
    std::vector<TrajectoryPoint> init_trajectory;
    generate_trajectory(final_qp_speed_profile, dp_trajectory, path_index2s, planning_start_point.time_stamped, init_trajectory);

    // 5.2轨迹拼接与上一周期轨迹缓存更新
    _stitcher.stitch_and_update(init_trajectory, final_trajectory);
    RCLCPP_INFO(LOG,"emplanner错误定位4");
    
    

    // ------------------------------------------------绘图------------------------------------------------
    // if (_plot_count % 20 == 0)
    // {
    //     matplot::figure(_path_polt_handle);
    //     matplot::cla();
    //     std::vector<double> init_dp_path_s,init_dp_path_l;
    //     std::vector<double> final_dp_path_s,final_dp_path_l;
    //     std::vector<double> init_qp_path_s, init_qp_path_l;

    //     for (size_t i = 0; i < dp_best_path.size(); i++)
    //     {
    //         init_dp_path_s.emplace_back(dp_best_path[i].s);    
    //         init_dp_path_l.emplace_back(dp_best_path[i].l);    
    //     }
        
    //     for (size_t i = 0; i < final_dp_path.size(); i++)
    //     {
    //         final_dp_path_s.push_back(final_dp_path[i].s);
    //         final_dp_path_l.push_back(final_dp_path[i].l);
    //         init_qp_path_s.emplace_back(init_qp_path[i].s);
    //         init_qp_path_l.emplace_back(init_qp_path[i].l);
    //     }
    //     // dp加密前
    //     // matplot::plot(init_dp_path_s, init_dp_path_l, "bs-");
    //     // dp加密后曲线
    //     matplot::plot(final_dp_path_s, final_dp_path_l,"bo-")->line_width(2);
    //     matplot::hold(true);
    //     // 凸空间
    //     matplot::plot(final_dp_path_s, final_dp_path_l_min, "r*-")->line_width(2);
    //     matplot::plot(final_dp_path_s, final_dp_path_l_max, "ro-")->line_width(2);
    //     // qp规划曲线
    //     matplot::plot(init_qp_path_s, init_qp_path_l, "go-")->line_width(2);
    //     // 绘制静态障碍物
    //     for (auto &&static_obs_sl_point : static_obstacle_frenet_coordinate)
    //     {
    //         matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1)->line_width(2);
    //         matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1)->line_width(2);
    //         matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1)->line_width(2);
    //         matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1)->line_width(2) ;
    //     }
    //     // matplot::legend({"DP planning ","up doundary","low boundary","QP planning"});

    // }
    // if (_plot_count % 20 == 0)
    // {
    //     matplot::figure(_trajectory_plot_handle);
    //     matplot::cla();
    //     // 动态规划轨迹
    //     std::vector<double> init_t_set, init_s_set;
    //     matplot::hold(true);
    //     for (auto &&st_point : dp_speed_profile)
    //     {
    //         init_t_set.emplace_back(st_point.t);
    //         init_s_set.emplace_back(st_point.s);
    //     }
    //     matplot::plot(init_t_set, init_s_set, "bo-")->line_width(2);

    //     // 二次规划轨迹
    //     std::vector<double> init_qp_speed_profile_t_set, init_qp_speed_profile_s_set;
    //     for (auto &&st_point : init_qp_speed_profile)
    //     {
    //         init_qp_speed_profile_t_set.emplace_back(st_point.t);
    //         init_qp_speed_profile_s_set.emplace_back(st_point.s);
    //     }
    //     matplot::plot(init_qp_speed_profile_t_set, init_qp_speed_profile_s_set, "g*-")->line_width(2);
        
        
    //     // 动态障碍物st图
    //     for (auto &&st_graph_node : dynamic_obs_st_graph)
    //     {
    //         matplot::line(st_graph_node.at("t_in"),st_graph_node.at("s_in"),
    //                       st_graph_node.at("t_out"), st_graph_node.at("s_out"))->line_width(2);
    //     }
        
        
    //     }
    
    // _plot_count++;
    // if (_plot_count == 1e10)
    // {
    //     _plot_count = 0;
    // }
 
    RCLCPP_INFO(LOG,"轨迹包含点数:%d",final_trajectory.size());
    RCLCPP_INFO(LOG,"emplanner错误定位5");
    _debug_data.compute_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - debug_start_time).count();

}

void EMPlanner::obstacle_fileter(std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object>& obstacles)
{
    auto LOG = rclcpp::get_logger("emplanner");
    // 将障碍物分成动态与静态障碍物
    _static_obstacles.clear();
    _dynamic_obstacles.clear();
    if(obstacles.empty())
    {
        return;
    }
    for (auto &&obs : obstacles)
    {
        if(obs.id == ego_state->id){continue;}// 障碍物不包含主车

        double v_obs = std::sqrt(std::pow(obs.twist.linear.x,2.0)+std::pow(obs.twist.linear.y,2.0)+std::pow(obs.twist.linear.z,2.0));// 障碍物速度
        Eigen::Vector2d host_to_obs(obs.pose.position.x - ego_state->x, obs.pose.position.y - ego_state->y);// 主车到障碍物的向量
        Eigen::Vector2d tau_host(std::cos(ego_state->heading), std::sin(ego_state->heading));
        Eigen::Vector2d nor_host(-std::sin(ego_state->heading), std::cos(ego_state->heading));
        double longitudinal_d = host_to_obs.dot(tau_host);// 纵向距离
        double lateral_d = host_to_obs.dot(nor_host);// 横向距离
        if(v_obs < 1e-2)// 静态障碍物，即使有加速度，一个规划周期是0.1s，障碍物以最大加速度加速也达不到很大的速度
        {
            if(longitudinal_d <= 60 && longitudinal_d >= -10 && lateral_d <= 10 && lateral_d >= -10)
            {
                _static_obstacles.push_back(obs);
            }
            else
            {
                continue;
            }
        }
        else// 动态障碍物 
        {
            if(longitudinal_d <= 60 && longitudinal_d >= -10 && lateral_d <= 20 && lateral_d >= -20)
            {
                _dynamic_obstacles.push_back(obs);
            }
            else
            {
                continue;
            }
        }
    }
}

