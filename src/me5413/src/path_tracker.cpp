#include "me5413/path_tracker.hpp"


namespace me5413 {


double SPEED_TARGET = 0.5;
double PID_Kp = 0.5;
double PID_Ki = 0.2; 
double PID_Kd = 0.2;
double STANLEY_K = 0.5;


PathTracker::PathTracker() : tf2_listener_(tf2_buffer_) {

    this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTracker::odomCallback, this);
    this->sub_local_path_ = nh_.subscribe("/me5413/planning/local_path", 1, &PathTracker::localPathCallback, this);
    this->pub_cmd_vel_    = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

    // Initialize frame names
    this->world_frame_ = "world";
    this->robot_frame_ = "base_link";

    this->speed_pid_ = controller::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd, 40);
}


void PathTracker::odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

    this->world_frame_ = odom->header.frame_id;
    this->robot_frame_ = odom->child_frame_id;
    this->odom_world_robot_ = *odom.get();

}


void PathTracker::localPathCallback(const nav_msgs::Path::ConstPtr &local_path) {

    this->tracking_goal_ = local_path->poses[11].pose;
    this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->tracking_goal_));

}


geometry_msgs::Twist PathTracker::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& tracking_goal) {
    /*
    Linear velocity is controlled by PID control (target speed - current speed) 
    Steering is controlled by a Stanley controller (combining heading and lateral errors)
    */

    // Calculate heading error
    tf2::Quaternion robot_heading, goal_heading;
    tf2::fromMsg(odom_robot.pose.pose.orientation, robot_heading);
    tf2::fromMsg(tracking_goal.orientation, goal_heading);
    const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(robot_heading);
    const tf2::Matrix3x3 m_goal  = tf2::Matrix3x3(goal_heading);

    double roll, pitch, yaw_robot, yaw_goal;
    m_robot.getRPY(roll, pitch, yaw_robot);
    m_goal.getRPY(roll, pitch, yaw_goal);

    const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);

    // Calculate lateral error
    tf2::Vector3 robot_position, goal_position;
    tf2::fromMsg(odom_robot.pose.pose.position, robot_position);
    tf2::fromMsg(tracking_goal.position, goal_position);
    // Vector from goal to robot
    const tf2::Vector3 V_goal_robot = robot_position - goal_position;

    const double angle_goal_robot   = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
    const double angle_diff         = angle_goal_robot - yaw_goal;
    const double lateral_error      = V_goal_robot.length() * std::sin(angle_diff);

    // Velocity
    tf2::Vector3 robot_vel;
    tf2::fromMsg(odom_robot.twist.twist.linear, robot_vel);

    const double velocity = robot_vel.length();

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x  = this->speed_pid_.calculate(SPEED_TARGET, velocity);
    cmd_vel.angular.z = computeStanelyControl(heading_error, lateral_error, velocity);

    return cmd_vel;
}


double PathTracker::computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity) {

  const double stanley_output = -1.0*(heading_error + std::atan2(STANLEY_K*cross_track_error, std::max(velocity, 0.3)));

  return std::min(std::max(stanley_output, -2.2), 2.2);

}



double PathTracker::unifyAngleRange(const double angle) {
    double angle_unified = angle;
    while (angle_unified > M_PI) {
        angle_unified -= 2 * M_PI;
    }
    while (angle_unified < -M_PI) {
        angle_unified += 2 * M_PI;
    }
    return angle_unified;
}

}   // namespace me5413