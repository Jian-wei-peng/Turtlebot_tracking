#ifndef PATH_TRACKER_HPP_
#define PATH_TRACKER_HPP_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "me5413/pid.hpp"


namespace me5413 {

class PathTracker {

    public:
        PathTracker();

    private:
        void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
        void localPathCallback(const nav_msgs::Path::ConstPtr &local_path);

        geometry_msgs::Twist computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal);

        double computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity);
        
        static double unifyAngleRange(const double angle);

    private:
        ros::NodeHandle nh_;
        ros::Timer timer_;

        ros::Subscriber sub_robot_odom_;
        ros::Subscriber sub_local_path_;

        ros::Publisher pub_cmd_vel_;

        nav_msgs::Odometry odom_world_robot_;
        geometry_msgs::Pose tracking_goal_;

        tf2_ros::Buffer tf2_buffer_;
        tf2_ros::TransformListener tf2_listener_;
        tf2_ros::TransformBroadcaster tf2_bcaster_;

        std::string world_frame_;
        std::string robot_frame_;

        // PID controller for speed control
        controller::PID speed_pid_;
};


} // namespace me5413

#endif  // PATH_TRACKER_HPP_
