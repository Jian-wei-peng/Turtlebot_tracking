#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include "pure_pursuit_controller/util/path_generator.hpp"
#include "pure_pursuit_controller/util/math_tool.hpp"

namespace pure_pursuit_controller {

class PurePursuit {
public:
    PurePursuit(ros::NodeHandle& nh);
    ~PurePursuit() = default;

    // 执行一次控制计算并立即发布 cmd_vel
    void ComputeAndPublish(const std::vector<PathPoint>& path_points);

private:
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    int FindLookaheadIndex(const std::vector<PathPoint>& path, const PathPoint& current_pose, double ld);

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;

    // odom
    PathPoint odom_pose_;
    double odom_linear_v_;
    double odom_angular_v_;
    bool odom_ready_;

    // control params
    double k_;     // lookahead gain
    double lf_;    // min lookahead
    double default_speed_;
    MathTool math_tool_;
};

} // namespace
#endif
