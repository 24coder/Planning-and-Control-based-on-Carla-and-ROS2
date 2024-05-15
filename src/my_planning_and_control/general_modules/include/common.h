#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"

struct TrajectoryPoint
{
    double x;
    double y;
    double heading;
    double kappa;
    double v;
    double a;
    double time_stamp;
};

struct PathPoint
{
    double x;
    double y;
    double heading;
    double kappa;
};

struct VehicleState
{
    double x;
    double y;
    double heading;
    double v;
    double a;
    double omega;
    double alpha;
    double time_stamp;
    bool flag_imu;
    bool flag_ode;
};

void Calculate_heading_and_kappa(std::shared_ptr<std::vector<PathPoint>> path);


