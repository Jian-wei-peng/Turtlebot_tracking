#include "pure_pursuit_controller/pure_pursuit.hpp"

namespace pure_pursuit_controller {

PurePursuit::PurePursuit(ros::NodeHandle& nh) : nh_(nh),
                                               odom_linear_v_(0.0),
                                               odom_angular_v_(0.0),
                                               odom_ready_(false),
                                               k_(1.0),
                                               lf_(0.1),
                                               default_speed_(0.5)
{
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &PurePursuit::OdomCallback, this);
}

void PurePursuit::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_pose_.x = msg->pose.pose.position.x;
    odom_pose_.y = msg->pose.pose.position.y;
    odom_pose_.yaw = tf::getYaw(msg->pose.pose.orientation);
    odom_linear_v_ = msg->twist.twist.linear.x;
    odom_angular_v_ = msg->twist.twist.angular.z;
    odom_ready_ = true;
}

/**
 * 查找预瞄点索引：
 * 1. 先找当前最近点索引
 * 2. 从该索引开始向前遍历（环形路径可回绕），直到找到第一个 distance >= ld 的点
 */
int PurePursuit::FindLookaheadIndex(const std::vector<PathPoint>& path, const PathPoint& current_pose, double ld) {
    if (path.empty()) return 0;

    int nearest_idx = math_tool_.GetMinDisIndex(current_pose, path);
    int idx = nearest_idx;

    // 循环查找第一个满足距离 >= ld 的点
    while (true) {
        double d = math_tool_.GetDistance(current_pose, path[idx]);
        if (d >= ld) {
            break;
        }
        idx++;
        if (idx >= static_cast<int>(path.size())) idx = 0; // loop
    }
    return idx;
}

void PurePursuit::ComputeAndPublish(const std::vector<PathPoint>& path_points) {
    if (!odom_ready_) {
        // 没有收到里程计，什么都不做
        return;
    }

    double v = odom_linear_v_;
    if (v < 1e-3) v = default_speed_;

    double ld = k_ * v + lf_;

    PathPoint current{odom_pose_.x, odom_pose_.y, odom_pose_.yaw};
    int ahead_idx = FindLookaheadIndex(path_points, current, ld);
    const PathPoint& ahead_pt = path_points[ahead_idx];

    double alpha = atan2(ahead_pt.y - current.y, ahead_pt.x - current.x) - current.yaw;

    // 差速轮模型角速度
    double omega = 2.0 * v * sin(alpha) / ld;

    geometry_msgs::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = omega;

    cmd_vel_pub_.publish(cmd);
}

}  // namespace
