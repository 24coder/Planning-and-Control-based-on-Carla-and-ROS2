#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "derived_object_msgs/msg/object.hpp"

#include "tf2_eigen/tf2_eigen.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

struct TrajectoryPoint
{
    double x;
    double y;
    double heading;
    double kappa;
    double v;
    double ax;
    double ay;
    rclcpp::Time time_stamped;
};

struct PathPoint
{
    double x;
    double y;
    double heading;
    double kappa;
};

struct FrenetPoint
{
    double s;
    double l;
    double s_dot;
    double l_dot;
    double l_prime;
    double s_dot_dot;
    double l_dot_dot;
    double l_prime_prime;
};

struct VehicleState
{
    double x;
    double y;
    double z;
    double heading;
    double v;
    double ax;
    double ay;
    double omega;
    double alpha;
    double time_stamp;
    uint32_t id;
    bool flag_imu;
    bool flag_ode;
    bool flag_info;
};


//计算路径的偏航角与曲率
void Calculate_heading_and_kappa(std::shared_ptr<std::vector<PathPoint>> path);
//计算投影点
void calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const Eigen::Vector2d point, int& match_point_index, PathPoint& projection_point);
void calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const TrajectoryPoint& trajectory_point, int& match_point_index, PathPoint& match_point, PathPoint& projection_point);//重载版本
//计算index2s表
std::vector<double> calculate_index_to_s(std::shared_ptr<std::vector<PathPoint>> path, std::shared_ptr<VehicleState> vehicle_state);
//笛卡尔坐标到frenet坐标的转换
//动态版本
FrenetPoint cartesion_to_frenet(const TrajectoryPoint& host, const PathPoint& projection_point, const PathPoint& match_point, std::vector<double> index2s, const int& match_point_index);
//静态版本，感觉不应区分，先不加入了


//FrenetPoint cartesion_to_frenet(const TrajectoryPoint& host, const PathPoint& projection_point, const PathPoint& match_point, std::vector<double> index2s, const int& match_point_index, const bool& flag);
TrajectoryPoint object_to_trajectory_point(const derived_object_msgs::msg::Object object);