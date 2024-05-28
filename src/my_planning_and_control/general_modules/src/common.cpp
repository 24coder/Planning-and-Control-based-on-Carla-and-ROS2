#include "common.h"


void Calculate_heading_and_kappa(std::shared_ptr<std::vector<PathPoint>> path)
{
    int n = path->size();
    std::vector<double> dx;
    std::vector<double> dy;
    std::vector<double> ds;
    dx.resize(n);
    dy.resize(n);
    ds.resize(n);
    for (int i = 0; i < (int)path->size(); i++)
    {
        if (i == 0)
        {
            dx[i] = path->at(i+1).x - path->at(i).x;
            dy[i] = path->at(i+1).y - path->at(i).y;
        }
        else if (i == (int)path->size()-1)
        {
            dx[i] = path->at(i).x - path->at(i-1).x;
            dy[i] = path->at(i).y - path->at(i-1).y;
        }
        else
        {
            dx[i] = (path->at(i+1).x - path->at(i-1).x)/2.0;
            dy[i] = (path->at(i+1).y - path->at(i-1).y)/2.0;
        }
        
        ds[i] = std::hypot(dx[i],dy[i]);
        path->at(i).heading = std::atan2(dy[i],dx[i]);
        
    }

    std::vector<double> dtheta;
    dtheta.resize(n);
    
    for (int i = 0; i < (int)path->size(); i++)
    {
        if (i == 0)
        {
            dtheta[i] = path->at(i+1).heading - path->at(i).heading;
        }
        else if (i == (int)path->size()-1)
        {
            dtheta[i] = path->at(i).heading - path->at(i-1).heading;
        }
        else
        {
            dtheta[i] = (path->at(i+1).heading - path->at(i-1).heading)/2.0;
        }
        
        //这里可能存在多值问题
        path->at(i).kappa = dtheta[i]/ds[i];
    }
    
}

void calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const Eigen::Vector2d point, int& match_point_index, PathPoint& projection_point)
{
    /*
    计算给定点在路径上的投影点
    1.输入参数
    */
    //1.搜索投影点
    double min_distance = std::numeric_limits<double>::max();
    size_t count_num = 0;
    for (size_t i = 0; i < path->size(); i++)
    {
        double distance = std::hypot(path->at(i).x - point[0], path->at(i).y - point[1]);
        if (distance < min_distance)
        {
            match_point_index = i;
            min_distance = distance;
            count_num = 0;
        }
        count_num ++;
        if (count_num > 30)
        {
            break;
        }
    }
    //2.由匹配点计算投影点
    Eigen::Vector2d tau_m; //匹配点切线向量
    Eigen::Vector2d d;//匹配点指向主车的向量 
    Eigen::Vector2d match_point;//匹配点位置向量
    tau_m << std::cos(path->at(match_point_index).heading),std::sin(path->at(match_point_index).heading);
    match_point << path->at(match_point_index).x, path->at(match_point_index).y;
    d = point - match_point;
    //2.1计算x，y
    auto v_pro = match_point + (d.dot(tau_m))*tau_m;
    projection_point.x = v_pro[0];
    projection_point.y = v_pro[1];
    //2.2计算heading和kappa
    projection_point.heading = path->at(match_point_index).heading + path->at(match_point_index).kappa *(d.dot(tau_m));
    projection_point.kappa = path->at(match_point_index).kappa;
 
}
//计算投影点的重载版本
void calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const TrajectoryPoint& trajectory_point, int& match_point_index,
                                 PathPoint& match_point, PathPoint& projection_point)
{
    double min_distance = std::numeric_limits<double>::max();
    size_t count_num = 0;
    for (size_t i = 0; i < path->size(); i++)
    {
        double distance = std::hypot(path->at(i).x - trajectory_point.x, path->at(i).y - trajectory_point.y);
        if (distance < min_distance)
        {
            match_point_index = i;
            min_distance = distance;
            count_num = 0;
        }
        count_num ++;
        if (count_num > 30)
        {
            break;
        }
    }
    //2.由匹配点计算投影点
    Eigen::Vector2d tau_m; //匹配点切线向量
    Eigen::Vector2d d;//匹配点指向主车的向量 
    Eigen::Vector2d v_match_point;//匹配点位置向量
    tau_m << std::cos(path->at(match_point_index).heading),std::sin(path->at(match_point_index).heading);
    v_match_point << path->at(match_point_index).x, path->at(match_point_index).y;
    d << trajectory_point.x - v_match_point[0], trajectory_point.y - v_match_point[1];
    //2.1计算x，y，赋值
    auto v_pro = v_match_point + (d.dot(tau_m))*tau_m;
    projection_point.x = v_pro[0];
    projection_point.y = v_pro[1];
    //2.2计算heading和kappa，赋值
    projection_point.heading = path->at(match_point_index).heading + path->at(match_point_index).kappa *(d.dot(tau_m));
    projection_point.kappa = path->at(match_point_index).kappa;

    //匹配点赋值
    match_point.x = v_match_point[0];
    match_point.y = v_match_point[1];
    match_point.heading = path->at(match_point_index).heading;
    match_point.kappa = path->at(match_point_index).kappa;
 
}

std::vector<double> calculate_index_to_s(std::shared_ptr<std::vector<PathPoint>> path, std::shared_ptr<VehicleState> vehicle_state)
{
    /*
    将路径转换为index2s表，加速后续程序计算
    */
    //1.计算投影点索引
    Eigen::Vector2d host_point(vehicle_state->x, vehicle_state->y);
    int match_point_index;
    PathPoint projection_point;
    calculate_projection_point(path,host_point,match_point_index,projection_point);

    //2.计算初始表，原点为第一个点
    std::vector<double> index2s;
    double distance = 0.0;
    index2s.push_back(distance);
    for (size_t i = 1; i < path->size(); i++)
    {
        distance += std::hypot(path->at(i).x - path->at(i-1).x, path->at(i).y - path->at(i-1).y);
        index2s.push_back(distance);
    }

    //3.将原点改至投影点
    //判断投影点在匹配点的前方还是后方,获取投影点所对应的s
    Eigen::Vector2d match_to_pro(projection_point.x - path->at(match_point_index).x,
                                projection_point.y - path->at(match_point_index).y);
    Eigen::Vector2d tau_match(std::cos(path->at(match_point_index).heading), std::sin(path->at(match_point_index).heading));
    double delta_s = 0;
    if(match_to_pro.dot(tau_match) > 0.0)//投影点在匹配点前方
    {
        delta_s = index2s[match_point_index] + match_to_pro.norm();
    }
    else if(match_to_pro.dot(tau_match) < 0.0)//投影点在匹配点后方
    {
        delta_s = index2s[match_point_index] - match_to_pro.norm();
    }
    else
    {
        delta_s = index2s[match_point_index];
    }

    //将原有的index2s表平移
    for (auto && element : index2s)
    {
        element -= delta_s;
    }

    //4.输出
    return index2s;

}

FrenetPoint cartesion_to_frenet(const TrajectoryPoint& host, const PathPoint& projection_point, const PathPoint& match_point, std::vector<double> index2s, const int& match_point_index)
{
    FrenetPoint frenet_point;
    //待转换点相关向量
    Eigen::Vector2d r_host(host.x, host.y);
    Eigen::Vector2d tau_host(std::cos(host.heading), std::sin(host.heading));
    Eigen::Vector2d nor_host(-std::sin(host.heading), std::cos(host.heading));
    Eigen::Vector2d a_host(host.ax, host.ay);
    double v_host = host.v;
    //投影点相关量
    Eigen::Vector2d r_pro(projection_point.x, projection_point.y);
    Eigen::Vector2d tau_pro(std::cos(projection_point.heading), std::sin(projection_point.heading));
    Eigen::Vector2d nor_pro(-std::sin(projection_point.heading), std::cos(projection_point.heading));
    double kr = projection_point.kappa;


    double l = (r_host - r_pro).dot(nor_pro);
    double s_dot = v_host * (tau_host.dot(tau_pro)) / (1 - kr * l);
    double l_dot = v_host * (tau_host.dot(nor_pro));
    double l_prime = l_dot / (s_dot + 1e-10);
    double s_dot_dot = a_host.dot(tau_pro)/(1-kr*l) + kr*s_dot*s_dot*l_prime/(1-kr*l) + s_dot*s_dot*kr*l_prime/(1-kr*l);
    double l_dot_dot = a_host.dot(nor_pro) - kr*(1-kr*l)*s_dot*s_dot;
    double l_prime_prime = (l_dot_dot - l_prime*s_dot_dot)/(s_dot*s_dot + 1e-10);
    double s = index2s[match_point_index];
    Eigen::Vector2d match_point_to_host(host.x - match_point.x, host.y - match_point.y);
    Eigen::Vector2d tau_match(std::cos(match_point.heading), std::sin(match_point.heading));
    if(match_point_to_host.dot(tau_match) > 0.0)
    {
        s += std::hypot(match_point.x - projection_point.x, match_point.y - projection_point.y);
    }
    else if(match_point_to_host.dot(tau_match) < 0.0)
    {
        s -= std::hypot(match_point.x - projection_point.x, match_point.y - projection_point.y);
    }

    frenet_point.s = s;
    frenet_point.l = l;
    frenet_point.s_dot = s_dot;
    frenet_point.l_dot = l_dot;
    frenet_point.l_prime = l_prime;
    frenet_point.s_dot_dot = s_dot_dot;
    frenet_point.l_dot_dot = l_dot_dot;
    frenet_point.l_prime_prime = l_prime_prime;

    return frenet_point;
}


//对象转换至轨迹点
TrajectoryPoint object_to_trajectory_point(const derived_object_msgs::msg::Object object)
{
    TrajectoryPoint trajectory_point;
    trajectory_point.x = object.pose.position.x;
    trajectory_point.y = object.pose.position.y;
    trajectory_point.v = std::sqrt(std::pow(object.twist.linear.x, 2.0) +
                                   std::pow(object.twist.linear.y, 2.0) +
                                   std::pow(object.twist.linear.z, 2.0));
    trajectory_point.kappa = 0.0;
    trajectory_point.ax = object.accel.linear.x;
    trajectory_point.ay = object.accel.linear.y;

    tf2::Quaternion tf2_q;
    tf2::fromMsg(object.pose.orientation,tf2_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll,pitch,yaw);
    trajectory_point.heading = yaw;

    return trajectory_point;
}