#include "pure_pursuit_controller/pure_pursuit.hpp"

namespace pure_pursuit_controller {

PurePursuit::PurePursuit(ros::NodeHandle& nh) : nh_(nh),
                                                odom_linear_v_(0.0),
                                                odom_angular_v_(0.0),
                                                odom_ready_(false)
{
    nh_.param("k", k_, 1.0); 
    nh_.param("lf", lf_, 0.1); 
    nh_.param("default_speed", default_speed_, 0.5);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub_    = nh_.subscribe<nav_msgs::Odometry>("/odom", 10, &PurePursuit::OdomCallback, this);

    dynamic_reconfigure::Server<PurePursuitConfig>::CallbackType cb;
    cb = boost::bind(&PurePursuit::DynamicReconfigCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
}

void PurePursuit::DynamicReconfigCallback(PurePursuitConfig &config, uint32_t level)
{
    k_  = config.k;
    lf_ = config.lf;
    default_speed_ = config.default_speed;

    ROS_INFO("Dynamic Reconfigure: k=%.2f, lf=%.2f, default_speed=%.2f", k_, lf_, default_speed_);
}

void PurePursuit::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_pose_.x    = msg->pose.pose.position.x;
    odom_pose_.y    = msg->pose.pose.position.y;
    odom_pose_.yaw  = tf::getYaw(msg->pose.pose.orientation);
    odom_linear_v_  = msg->twist.twist.linear.x;
    odom_angular_v_ = msg->twist.twist.angular.z;
    odom_ready_     = true;
}

int PurePursuit::FindLookaheadIndex(const std::vector<PathPoint>& path,
                                    const PathPoint& current_pose, 
                                    double ld)
{
    if (path.empty()) return -1;

    // Find the nearest point index first
    int nearest_idx = math_tool_.GetMinDisIndex(current_pose, path);

    // Search for the first point beyond lookahead distance
    for (size_t i = nearest_idx; i < path.size(); ++i) {
        if (math_tool_.GetDistance(current_pose, path[i]) >= ld) {
            return i;
        }
    }

    // If no point is found, return the last point
    return path.size() - 1;
}

void PurePursuit::ComputeAndPublish(const std::vector<PathPoint>& path_points) {
    if (!odom_ready_) {
        ROS_INFO("Waiting for /odom message...");
        return;
    }

    double v = odom_linear_v_;
    if (v < 1e-3) v = default_speed_;

    // Lookahead distance
    double ld = k_ * v + lf_;

    // Find lookahead point
    PathPoint current{odom_pose_.x, odom_pose_.y, odom_pose_.yaw};
    int ahead_idx = FindLookaheadIndex(path_points, current, ld);
    const PathPoint& ahead_pt = path_points[ahead_idx];

    // Relative angle of lookahead point
    double alpha = atan2(ahead_pt.y - current.y, ahead_pt.x - current.x) - current.yaw;
    while (alpha  >  M_PI) alpha -= 2*M_PI;
    while (alpha <= -M_PI) alpha += 2*M_PI;

    // Angular velocity
    double omega = 2.0 * v * sin(alpha) / ld;

    // Publish cmd_vel
    geometry_msgs::Twist cmd;
    cmd.linear.x  = v;
    cmd.angular.z = omega;
    cmd_vel_pub_.publish(cmd);
}

}  // namespace pure_pursuit_controller
