#ifndef PATH_PUBLISHER_HPP_
#define PATH_PUBLISHER_HPP_

#include <string>
#include <vector>
#include <limits>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2/convert.h>                     // tf2::fromMsg, tf2::toMsg    
#include <tf2/LinearMath/Matrix3x3.h>        // tf2::Matrix3x3
#include <tf2/LinearMath/Transform.h>        // tf2::Transform
#include <tf2/LinearMath/Quaternion.h>       // tf2::Quaternion
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace me5413 {

class PathPublisher {
    public:
        PathPublisher();
        ~PathPublisher() = default;

    private:
        void timerCallback(const ros::TimerEvent &);

        void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);

        void publishGlobalPath();

        void publishLocalPath(const geometry_msgs::Pose &robot_current_pose, const int num_points_before_target, const int num_points_after_target);

        std::vector<geometry_msgs::PoseStamped> createGlobalPath(const double A, const double B, const double t_res);
        
        int nextWaypoint(const geometry_msgs::Pose &robot_current_pose, const nav_msgs::Path &global_path, const int id_start);

        int closestWaypoint(const geometry_msgs::Pose &robot_current_pose, const nav_msgs::Path &global_path, const int id_start);

        double getYawFromOrientation(const geometry_msgs::Quaternion &orientation);

        tf2::Transform convertPoseToTransform(const geometry_msgs::Pose &pose);

        std::pair<double, double> calculatePoseError(const geometry_msgs::Pose &robot_current_pose, const geometry_msgs::Pose &tracking_goal);

        static double unifyAngleRange(const double angle);

        static bool isLegal(const double x);

    private:
        ros::NodeHandle nh_;
        ros::Timer timer_;

        ros::Subscriber sub_robot_odom_;

        ros::Publisher pub_global_path_;
        ros::Publisher pub_local_path_;

        nav_msgs::Path global_path_msg_;
        nav_msgs::Path local_path_msg_;

        tf2_ros::Buffer tf2_buffer_;
        tf2_ros::TransformListener tf2_listener_;
        tf2_ros::TransformBroadcaster tf2_bcaster_;

        std::string world_frame_;
        std::string robot_frame_;

        // Robot odometry in the world frame
        nav_msgs::Odometry odom_world_robot_;

        // Goal pose for the robot in the world frame, used by the tracking controller
        geometry_msgs::Pose tracking_goal_;
        
        int current_id_;
        std_msgs::Float32 abs_position_error_;
        std_msgs::Float32 abs_heading_error_;
        std_msgs::Float32 abs_speed_error_;

};  // class PathPublisher

} // namespace me5413


#endif  // PATH_PUBLISHER_HPP_