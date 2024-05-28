#include "emplanner.h"

auto LOG = rclcpp::get_logger("emplanner");

EMPlanner::EMPlanner() : Node("emlpanner")
{
    _get_waypoint_client = this->create_client<carla_waypoint_types::srv::GetWaypoint>(
    "/carla_waypoint_publisher/ego_vehicle/get_waypoint"
    );

    //路径决策代价初始化
    _weight_coefficients.path_dp_w_ref = 100.0;
    _weight_coefficients.path_dp_w_dl = 1000.0;
    _weight_coefficients.path_dp_w_ddl = 100.0;
    _weight_coefficients.path_dp_w_dddl = 10.0;
    _weight_coefficients.path_dp_w_obs = 1e5;

    //路径规划代价初始化
    _weight_coefficients.path_qp_w_ref = 100.0;
    _weight_coefficients.path_qp_w_dl = 100.0;
    _weight_coefficients.path_qp_w_ddl = 100.0;
    _weight_coefficients.path_qp_w_dddl = 100.0;
    _weight_coefficients.path_qp_w_mid = 1000.0;
    _weight_coefficients.path_qp_w_l_end = 10.0;
    _weight_coefficients.path_qp_w_dl_end = 10.0;
    _weight_coefficients.path_qp_w_ddl_end = 10.0;

    //绘图
    _path_polt_handle = matplot::figure();
    _plot_count = 0;

    //路径二次规划
    _path_qp_solver.settings()->setWarmStart(true);
    

}




void EMPlanner::planning_run_step(std::shared_ptr<std::vector<PathPoint>> reference_line, std::shared_ptr<VehicleState> ego_state, 
                        const std::vector<derived_object_msgs::msg::Object> obstacles, rclcpp::Time current_time)
{   _current_time = current_time;
    //1障碍物处理
    //1.1区别动态与静态障碍物，存储在类成员变量里
    obstacle_fileter(ego_state, obstacles);
    //2.2障碍物坐标转换
    auto reference_index2s = calculate_index_to_s(reference_line,ego_state);
    // for (auto &&s : reference_index2s)
    // {
    //     RCLCPP_INFO(LOG,"index2s表%.3f",s);
    // }
    
    std::vector<FrenetPoint> static_obstacle_frenet_coordinate;
    std::vector<FrenetPoint> dynamic_obstacle_frenet_coordinate;
    if(!_static_obstacles.empty())
    {
        for (auto &&static_obs : _static_obstacles)
        {
            auto obs_trajectory = object_to_trajectory_point(static_obs);//将object转化为轨迹点
            PathPoint match_point, projection_poin;//匹配点，投影点
            int match_point_index;//匹配点索引
            calculate_projection_point(reference_line,obs_trajectory,match_point_index,match_point,projection_poin);
            auto obs_frenet_coordinate = cartesion_to_frenet(obs_trajectory,projection_poin,match_point,reference_index2s,match_point_index);
            static_obstacle_frenet_coordinate.push_back(obs_frenet_coordinate);
        }

        for (auto && s_obs_frenet : static_obstacle_frenet_coordinate)
        {
            RCLCPP_INFO(LOG,"静态障碍物信息(%.3f,%.3f)",s_obs_frenet.s,s_obs_frenet.l);
        }
        
        
    }
    if(!_dynamic_obstacles.empty())
    {
        for (auto &&dynamic_obs : _dynamic_obstacles)
        {
            auto obs_trajectroy = object_to_trajectory_point(dynamic_obs);
            PathPoint match_point, projection_poin;//匹配点，投影点
            int match_point_index;//匹配点索引
            calculate_projection_point(reference_line,obs_trajectroy,match_point_index,match_point,projection_poin);
            auto obs_frenet_coordinate = cartesion_to_frenet(obs_trajectroy,projection_poin,match_point,reference_index2s,match_point_index);
            dynamic_obstacle_frenet_coordinate.push_back(obs_frenet_coordinate);
        }   
    }

    //2.确定规划起点并投影到frenet坐标系
    auto planning_start_point = calculate_planning_start_point(ego_state);
    PathPoint psp_match_point, psp_projection_point;
    int psp_match_point_index;
    calculate_projection_point(reference_line,planning_start_point,psp_match_point_index,psp_match_point,psp_projection_point);
    auto planning_start_point_frenet_coordinate = cartesion_to_frenet(planning_start_point,psp_projection_point,psp_match_point,reference_index2s,psp_match_point_index);


    //3.轨迹规划
    //3.1获取车道宽度
    double lane_width = 4.0;
    double car_width = 2.0;

    // auto get_waypoint_request = std::make_shared<carla_waypoint_types::srv::GetWaypoint::Request>();
    // get_waypoint_request->location.x = ego_state->x;
    // get_waypoint_request->location.y = ego_state->y;
    // get_waypoint_request->location.z = ego_state->z;
    // auto response = _get_waypoint_client->async_send_request(get_waypoint_request);
    // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),response) == rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     lane_width = response.get()->waypoint.lane_width;

    //         RCLCPP_INFO(this->get_logger(),"请求成功,车道宽度为:%.3f",lane_width);
    // }

    //3.2dp路径规划
    //3.2.1获取采用点
    std::vector<std::vector<FrenetPoint>> path_dp_sample;
    get_path_dp_sample(path_dp_sample,planning_start_point_frenet_coordinate,10.0,6,1.0,7);

    // for (size_t level = 1; level <= path_dp_sample.size() ; level++)
    // {
    //     RCLCPP_INFO(this->get_logger(),"-------第%d列采样点--------",level);
    //     auto vector_point = path_dp_sample[level-1];
    //     for (auto &&point : vector_point)
    //     {
    //         RCLCPP_INFO(this->get_logger(),"采样点坐标(%.3f,%.3f)",point.s,point.l);
    //     }   
    // }

    //3.2.2获取最小代价
    std::vector<std::vector<DpPathNode>> path_dp_node_table;
    path_dp_node_table.emplace_back();
    path_dp_node_table.back().emplace_back(planning_start_point_frenet_coordinate,0.0,nullptr);
    auto path_dp_sample_front = path_dp_sample.front().front();
    for (size_t level = 1; level < path_dp_sample.size(); level++)
    {
        path_dp_node_table.emplace_back();
        if (level == 1)//第一列
        {   
            //遍历第一列每一个节点，与起始点计算代价，算出来的代价直接是最小代价
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
            //对于不是第一列的点，要对该列的每一个点与前一列的每一个计算代价
            for (auto && current_sample_point : path_dp_sample[level])
            {
                double min_cost = std::numeric_limits<double>::max();
                size_t index;
                for (size_t i = 0 ; i < path_dp_node_table[level -1].size(); i++)
                {
                    //当前代价 = 前一列点的最小代价 + 前一列点到当前点的代价
                    double cur_cost =
                    path_dp_node_table[level-1][i].min_cost +
                    calculate_dp_cost(path_dp_node_table[level-1][i].sl_point, current_sample_point, static_obstacle_frenet_coordinate, _weight_coefficients);
                    if ( cur_cost < min_cost )
                    {
                        //记录最小代价与相应的前一列点的索引
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

    //3.2.3利用path_dp_node_table找出最优路径
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
    auto path_useful_node = path_dp_node_table.back()[path_min_cost_index];//最后一列的最小代价节点
    
    std::vector<FrenetPoint> dp_best_path;//存放dp产生的最优路径
    dp_best_path.emplace_back(path_useful_node.sl_point);
    while (true)
    {
        //由最后一列的最小代价节点中的指针，一步一步向前推，把整条路径找出来给
        path_useful_node = *(path_useful_node.pre_node);
        dp_best_path.emplace(dp_best_path.begin(),path_useful_node.sl_point);
        if(dp_best_path.front().s == planning_start_point_frenet_coordinate.s &&
           dp_best_path.front().l == planning_start_point_frenet_coordinate.l)
        {
            break;
        }
    }

    //3.2.4对动态规划出来的路径进行加密
    std::vector<FrenetPoint> final_dp_path;
    increased_dp_path(dp_best_path, 1.0, final_dp_path);

    //3.2.5获取凸空间
    //现在假定静态障碍物都是车，一般化情况应该是根据相应的障碍物类型获得相应的边框
    std::vector<double> final_dp_path_l_max, final_dp_path_l_min;
    double safe_distance = 0.0;
    double road_up_boundary = 1.5 * lane_width  - safe_distance;
    double road_low_boundary = -(0.5 * lane_width  - safe_distance);
    generate_convex_space(final_dp_path, road_up_boundary, road_low_boundary, static_obstacle_frenet_coordinate, final_dp_path_l_max, final_dp_path_l_min);


    //3.3qp路径规划
    std::vector<FrenetPoint> init_qp_path;
    double l_desire = 0.0;
    double dl_desire = 0.0;
    double ddl_desire = 0.0;
    path_QP_planning(final_dp_path, final_dp_path_l_min, final_dp_path_l_max, l_desire, dl_desire, ddl_desire, _weight_coefficients, init_qp_path);
    //3绘图
    if (_plot_count % 5 == 0)
    {
        matplot::figure(_path_polt_handle);
        matplot::cla();
        std::vector<double> init_dp_path_s,init_dp_path_l;
        std::vector<double> final_dp_path_s,final_dp_path_l;
        std::vector<double> init_qp_path_s, init_qp_path_l;

        for (size_t i = 0; i < dp_best_path.size(); i++)
        {
            init_dp_path_s.emplace_back(dp_best_path[i].s);    
            init_dp_path_l.emplace_back(dp_best_path[i].l);    
        }
        
        for (size_t i = 0; i < final_dp_path.size(); i++)
        {
            final_dp_path_s.push_back(final_dp_path[i].s);
            final_dp_path_l.push_back(final_dp_path[i].l);
            init_qp_path_s.emplace_back(init_qp_path[i].s);
            init_qp_path_l.emplace_back(init_qp_path[i].l);

        }
        //dp加密前
        matplot::plot(init_dp_path_s, init_dp_path_l, "bs-");
        //dp加密后曲线
        matplot::plot(final_dp_path_s, final_dp_path_l,"bo-");
        matplot::hold(true);
        //凸空间
        matplot::plot(final_dp_path_s, final_dp_path_l_min, "r*-");
        matplot::plot(final_dp_path_s, final_dp_path_l_max, "r*-");
        //qp规划曲线
        matplot::plot(init_qp_path_s, init_qp_path_l, "go-");
        //绘制静态障碍物
        for (auto &&static_obs_sl_point : static_obstacle_frenet_coordinate)
        {
            matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1);
            matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1);
            matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1);
            matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1);
        }
        

        
    }
    _plot_count++;
    if (_plot_count == 1e10)
    {
        _plot_count = 0;
    }


    



    
    //****************上一周期轨迹赋值
    //_previous_trajectory = *********;
}




//获取规划起点与拼接轨迹
TrajectoryPoint EMPlanner::calculate_planning_start_point(std::shared_ptr<VehicleState> ego_state)
{
    _switch_trajectory.clear();
    TrajectoryPoint planning_start_point;
    //如果第一次运行
    double double_delta_T = 0.1;
    rclcpp::Duration delta_T(std::chrono::milliseconds((int)double_delta_T*1000));
    if(_previous_trajectory.empty())
    {
        planning_start_point.x = ego_state->x;
        planning_start_point.y = ego_state->y;
        planning_start_point.v = 0.0;
        planning_start_point.heading = ego_state->heading;
        planning_start_point.ax = 0.0;
        planning_start_point.ay = 0.0;
        planning_start_point.kappa = 0.0;
        planning_start_point.time_stamped = _current_time + delta_T;
    }
    else//不是第一次运行，已经有了上一周期规划的轨迹
    {
        //计算主车位置与目标点之间的误差
        size_t current_time_index = -1;
        if (_current_time <= _previous_trajectory[0].time_stamped)
        {
            current_time_index  = 0;
        }
        for (size_t i = 0; i < _previous_trajectory.size()-2; i++)//获取当前时刻在上一周期轨迹对应的时间索引
        {
            if (_current_time > _previous_trajectory[i].time_stamped && _current_time <= _previous_trajectory[i+1].time_stamped)
            {
                if((_current_time - _previous_trajectory[i].time_stamped)<=(_previous_trajectory[i+1].time_stamped - _current_time))
                {
                    current_time_index  = i;
                }
                else
                {
                    current_time_index = i+1;
                }
            }
        }
        auto target_point = _previous_trajectory[current_time_index];
        Eigen::Vector2d tau_target_point(std::cos(target_point.heading), std::sin(target_point.heading));
        Eigen::Vector2d nor_target_point(-std::sin(target_point.heading), std::cos(target_point.heading));
        Eigen::Vector2d host_to_target_point(target_point.x - ego_state->x, target_point.y - ego_state->y);
        double error_longitudional = std::abs(host_to_target_point.dot(tau_target_point));//纵向误差
        double error_lateral = std::abs(host_to_target_point.dot(nor_target_point));//横向误差

        if(error_lateral < 0.5 && error_longitudional < 1.5)//主车实际位置与目标点差距不大
        {
            //在上一周期轨迹上搜索规划起点
            size_t start_time_index = -1;
            rclcpp::Time start_time = _current_time + delta_T;
            if(start_time <= _previous_trajectory[0].time_stamped)
            {
                start_time_index = 0;
            }
            for (size_t i = 0; i < _previous_trajectory.size()-2; i++)
            {
                if (start_time > _previous_trajectory[i].time_stamped && start_time <= _previous_trajectory[i+1].time_stamped)
                {
                    if((start_time - _previous_trajectory[i].time_stamped)<=(_previous_trajectory[i+1].time_stamped - start_time))
                    {
                        start_time_index  = i;
                    }
                    else
                    {
                        start_time_index = i+1;
                    }
                }
            }

            planning_start_point.x = _previous_trajectory[start_time_index].x;
            planning_start_point.y = _previous_trajectory[start_time_index].y;
            planning_start_point.v = _previous_trajectory[start_time_index].v;
            planning_start_point.heading = _previous_trajectory[start_time_index].heading;
            planning_start_point.ax = _previous_trajectory[start_time_index].ax;
            planning_start_point.ay = _previous_trajectory[start_time_index].ay;
            //这里还要一个值得商讨的问题，实际使用current_time+delta_T还是上一周期上start_time_index所对应轨迹点的时间
            planning_start_point.time_stamped = start_time;

            //这个时候还要拼接上一段轨迹,向前拼20个点
            if(start_time_index >= 19)
            {
                for (size_t i = 0; i <= 19; i++)
                {
                    _switch_trajectory.push_back(_previous_trajectory[i]);//这个排列顺序是序号越大，时间越靠后的
                }
            }
            else if(start_time_index > 0)
            {
                for (size_t i = 0; i < start_time_index; i++)
                {
                    _switch_trajectory.push_back(_previous_trajectory[i]);//这个排列顺序是序号越大，时间越靠后的 
                }
                
            }
            

        }
        else//主车实际位置与目标点差距很大
        {
            //用动力学方程外推,认为在着100ms中加速度变化很小
            planning_start_point.ax = ego_state->ax;
            planning_start_point.ay = ego_state->ay;
            double ego_vx = ego_state->v * std::cos(ego_state->heading);
            double ego_vy = ego_state->v * std::sin(ego_state->heading);
            double planning_start_point_vx = ego_vx + ego_state->ax * double_delta_T;
            double planning_start_point_vy = ego_vy + ego_state->ay * double_delta_T;
            planning_start_point.v = std::hypot(planning_start_point_vx,planning_start_point_vy);
            planning_start_point.heading = std::atan2(planning_start_point_vy,planning_start_point_vx);
            planning_start_point.x = ego_state->x + ego_vx*double_delta_T + 0.5*ego_state->ax*double_delta_T*double_delta_T;
            planning_start_point.y = ego_state->y + ego_vy*double_delta_T + 0.5*ego_state->ay*double_delta_T*double_delta_T;
            planning_start_point.time_stamped = _current_time + delta_T;
        }
        

    }

    return planning_start_point;
}


void EMPlanner::obstacle_fileter(std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object> obstacles)
{
    auto LOG = rclcpp::get_logger("emplanner");
    //将障碍物分成动态与静态障碍物
    _static_obstacles.clear();
    _dynamic_obstacles.clear();
    if(obstacles.empty())
    {
        return;
    }
    for (auto &&obs : obstacles)
    {
        if(obs.id == ego_state->id){continue;}//障碍物不包含主车

        double v_obs = std::sqrt(std::pow(obs.twist.linear.x,2.0)+std::pow(obs.twist.linear.y,2.0)+std::pow(obs.twist.linear.z,2.0));//障碍物速度
        Eigen::Vector2d host_to_obs(obs.pose.position.x - ego_state->x, obs.pose.position.y - ego_state->y);//主车到障碍物的向量
        Eigen::Vector2d tau_host(std::cos(ego_state->heading), std::sin(ego_state->heading));
        Eigen::Vector2d nor_host(-std::sin(ego_state->heading), std::cos(ego_state->heading));
        double longitudinal_d = host_to_obs.dot(tau_host);//纵向距离
        double lateral_d = host_to_obs.dot(nor_host);//横向距离
        if(v_obs < 1e-2)//静态障碍物，即使有加速度，一个规划周期是0.1s，障碍物以最大加速度加速也达不到很大的速度
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
        else//动态障碍物 
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

