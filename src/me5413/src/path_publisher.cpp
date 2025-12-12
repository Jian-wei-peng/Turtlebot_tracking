#include "me5413/path_publisher.hpp"

namespace me5413 {

/// ------------------------------------------------------------------------------------------ //

double TRACK_A_AXIS = 8.0;
double TRACK_B_AXIS = 8.0;
double TRACK_WP_NUM = 500;
double NUM_POINTS_BEFORE_TARGET = 10;
double NUM_POINTS_AFTER_TARGET  = 50;
double SPEED_TARGET = 0.5;

// ------------------------------------------------------------------------------------------ //

// PathPublisher::PathPublisher() : tf2_listener_(tf2_buffer_) {
PathPublisher::PathPublisher() : tf2_listener_(tf2_buffer_) {

    // Set up a timer to periodically (0.1s) publish the path
    this->timer_ = this->nh_.createTimer(ros::Duration(0.1), &PathPublisher::timerCallback, this);

    // Set up subscriber to robot odometry
    this->sub_robot_odom_ = this->nh_.subscribe("/gazebo/ground_truth/state",
                                                1,
                                                &PathPublisher::odomCallback,
                                                this);

    // Set up publishers for global and local paths
    this->pub_global_path_ = nh_.advertise<nav_msgs::Path>("/me5413/planning/global_path", 1);
    this->pub_local_path_  = nh_.advertise<nav_msgs::Path>("/me5413/planning/local_path", 1);

    // Initialize frame names
    this->world_frame_ = "world";
    this->robot_frame_ = "base_link";

    // Create and store the global path ( nav_msgs::Path contains header and poses )
    this->global_path_msg_.header.frame_id = this->world_frame_;
    this->global_path_msg_.poses = createGlobalPath(TRACK_A_AXIS, TRACK_B_AXIS, 1.0/TRACK_WP_NUM);
    
    this->abs_position_error_.data = 0.0;  
    this->abs_heading_error_.data  = 0.0;

    this->current_id_ = 0;

}   


void PathPublisher::timerCallback(const ros::TimerEvent &) {

    // Publish the global and local paths at each timer event
    publishGlobalPath();
    publishLocalPath(this->odom_world_robot_.pose.pose, NUM_POINTS_BEFORE_TARGET, NUM_POINTS_AFTER_TARGET);

    // Calculate the absolute errors between the robot's current pose and the tracking goal
    const std::pair<double, double> abs_errors = calculatePoseError(this->odom_world_robot_.pose.pose, this->tracking_goal_);
    this->abs_position_error_.data = abs_errors.first;
    this->abs_heading_error_.data  = abs_errors.second;

    // Calculate speed error
    tf2::Vector3 velocity;
    tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, velocity);
    this->abs_speed_error_.data = velocity.length() - SPEED_TARGET;

}


void PathPublisher::odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

    this->world_frame_ = odom->header.frame_id;
    this->robot_frame_ = odom->child_frame_id;
    this->odom_world_robot_ = *odom.get();

    // Convert nav_msgs::Odometry to tf2::Transform for coordinate transformations or TF broadcasting
    const tf2::Transform T_world_robot = convertPoseToTransform(this->odom_world_robot_.pose.pose); 

    // Inverse transform to get robot to world
    const tf2::Transform T_robot_world = T_world_robot.inverse();

    // Prepare the TransformStamped message for broadcasting
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp    = ros::Time::now();
    transformStamped.header.frame_id = this->robot_frame_;
    transformStamped.child_frame_id  = this->world_frame_;
    // Set translation and rotation
    transformStamped.transform.translation = tf2::toMsg(T_robot_world.getOrigin());
    transformStamped.transform.rotation    = tf2::toMsg(T_robot_world.getRotation());

    // Broadcast the transform from robot to world
    this->tf2_bcaster_.sendTransform(transformStamped);

}


std::vector<geometry_msgs::PoseStamped> PathPublisher::createGlobalPath(const double A, const double B, const double t_res) {

    std::vector<geometry_msgs::PoseStamped> path_poses;
    const double t_increament = t_res * 2 * M_PI;
    
    // Generate waypoints along the figure-eight trajectory
    for (double t = 0.0; t <= 2 * M_PI; t += t_increament) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = A * std::sin(t);
        pose.pose.position.y = B * std::sin(t) * std::cos(t);
        pose.pose.position.z = 0.0;
        path_poses.push_back(pose);
    }

    // Calculate orientations for each waypoint
    tf2::Quaternion q;
    for (int i = 0; i < path_poses.size(); i++) {
        // Calculate yaw based on the direction to the next waypoint
        const double x_d = path_poses[i + 1].pose.position.x - path_poses[i].pose.position.x;
        const double y_d = path_poses[i + 1].pose.position.y - path_poses[i].pose.position.y;
        const double yaw = std::atan2(y_d, x_d);
        // Transform yaw to quaternion
        q.setRPY(0.0, 0.0, yaw);
        q.normalize();
        path_poses[i].pose.orientation = tf2::toMsg(q);
    }
    // Handle the last waypoint orientation (pointing to the previous waypoint)
    path_poses.back().pose.orientation = tf2::toMsg(q);

    return path_poses;
}


void PathPublisher::publishGlobalPath() {
    // Update the header timestamp and publish the global path
    this->global_path_msg_.header.stamp = ros::Time::now();
    this->pub_global_path_.publish(this->global_path_msg_);
}


void PathPublisher::publishLocalPath(const geometry_msgs::Pose &robot_current_pose, const int num_points_before_target, const int num_points_after_target) {
    /** 
     * Clip the local path segment around the robot's current position and publish to the tracker 
    **/ 

    // Find the next waypoint index based on the robot's current pose
    int id_next = nextWaypoint(robot_current_pose, this->global_path_msg_, this->current_id_);

    if (this->global_path_msg_.poses.empty()) { 
        // Judge if the global path is empty
        ROS_WARN("Global Path not published yet, waiting");
    } else if (id_next >= this->global_path_msg_.poses.size() - 1) { 
        // Judge if the robot has reached the end of the track
        ROS_WARN("Robot has reached the end of the track, please restart");
    } else {
        // Get the local path segment around the next waypoint
        this->current_id_ = std::max(this->current_id_, id_next - 1);
        int id_start = std::max(id_next - num_points_before_target, 0);
        int id_end   = std::min(id_next + num_points_after_target, int(this->global_path_msg_.poses.size() - 1));
        auto start   = this->global_path_msg_.poses.begin() + id_start;
        auto end     = this->global_path_msg_.poses.begin() + id_end + 1;   // +1 to include the endpoint

        // Update the local path message
        this->local_path_msg_.header.stamp    = ros::Time::now();
        this->local_path_msg_.header.frame_id = this->world_frame_;
        this->local_path_msg_.poses.assign(start, end);

        // Update the goal pose for the robot (used by the tracker)
        // Note: local_path_msg_ has been clipped by assign(), so index [num_points_before_target] corresponds to [id_next], which is the target waypoint
        this->tracking_goal_ = this->local_path_msg_.poses[num_points_before_target].pose;

        // Publish the local path
        this->pub_local_path_.publish(this->local_path_msg_);
    }

}


int PathPublisher::nextWaypoint(const geometry_msgs::Pose &robot_current_pose, const nav_msgs::Path &global_path, const int id_start) {

    // Find the closest waypoint first
    int id_closest = closestWaypoint(robot_current_pose, global_path, id_start);

    // Calculate the heading from the robot to the closest waypoint
    double yaw_T_robot_wp = atan2((global_path.poses[id_closest].pose.position.y - robot_current_pose.position.y),
                                  (global_path.poses[id_closest].pose.position.x - robot_current_pose.position.x));

    // Get the robot's current yaw (Note: robot_current_pose and global_path must be in the same coordinate system)
    const double yaw_robot  = getYawFromOrientation(robot_current_pose.orientation);

    // Calculate the angle difference between robot's heading and the heading to the waypoint
    const double angle      = std::fabs(yaw_robot - yaw_T_robot_wp);
    const double angle_norm = std::min(2 * M_PI - angle, angle);

    // Determine if the closest waypoint is ahead or behind the robot
    //      if behind (angle_norm > π/2), increment to get the next waypoint
    //      if ahead  (angle_norm <= π/2), keep the closest waypoint
    if (angle_norm > M_PI / 2) {
        id_closest++;
    }

    return id_closest;
}


int PathPublisher::closestWaypoint(const geometry_msgs::Pose &robot_current_pose, const nav_msgs::Path &global_path, const int id_start) {

    double min_dist = std::numeric_limits<double>::max();
    int id_closest  = id_start;

    for (int i = id_start; i < global_path.poses.size(); i++) {

        const double dist = std::hypot(
            robot_current_pose.position.x - global_path.poses[i].pose.position.x, 
            robot_current_pose.position.y - global_path.poses[i].pose.position.y
        );

        if (dist <= min_dist) {
            min_dist   = dist;
            id_closest = i;
        } else {
            break;
        }
    }

    return id_closest;
}


double PathPublisher::getYawFromOrientation(const geometry_msgs::Quaternion &orientation) {

    // Create a tf2::Quaternion object
    tf2::Quaternion q;

    // Convert geometry_msgs::Quaternion to tf2::Quaternion
    tf2::fromMsg(orientation, q);

    // Construct a 3×3 rotation matrix using quaternions
    q.normalize();
    const tf2::Matrix3x3 m(q);

    // Extract roll, pitch, and yaw from the rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}


tf2::Transform PathPublisher::convertPoseToTransform(const geometry_msgs::Pose &pose) {
    
    // Create a tf2::Transform object
    tf2::Transform T;

    // Set the origin (translation) of the transform (tf2::Vector3 is a 3D vector class used to represent translation)
    T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));

    // Create a tf2::Quaternion object
    tf2::Quaternion q;

    // Set the rotation (orientation) of the transform
    q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    q.normalize();
    T.setRotation(q);

    return T;

}


std::pair<double, double> PathPublisher::calculatePoseError(const geometry_msgs::Pose &robot_current_pose, const geometry_msgs::Pose &tracking_goal) {

    // Calculate position error (Euclidean distance in the XY plane)
    const double position_error = std::hypot(
        robot_current_pose.position.x - tracking_goal.position.x,
        robot_current_pose.position.y - tracking_goal.position.y
    );

    // Calculate orientation error (difference in yaw angles)
    tf2::Quaternion q_robot, q_waypoint;
    tf2::fromMsg(robot_current_pose.orientation, q_robot);
    tf2::fromMsg(tracking_goal.orientation, q_waypoint);

    const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
    const tf2::Matrix3x3 m_goal  = tf2::Matrix3x3(q_waypoint);

    double roll, pitch, yaw_robot, yaw_waypoint;
    m_robot.getRPY(roll, pitch, yaw_robot);
    m_goal.getRPY(roll, pitch, yaw_waypoint);

    // Calculate heading error in degrees, unified to [-180, 180]
    const double heading_error = unifyAngleRange(yaw_robot - yaw_waypoint) / M_PI * 180.0;

    return std::pair<double, double>(position_error, isLegal(heading_error) ? heading_error : 0.0);
}


double PathPublisher::unifyAngleRange(const double angle) {
    double angle_unified = angle;
    while (angle_unified > M_PI) {
        angle_unified -= 2 * M_PI;
    }
    while (angle_unified < -M_PI) {
        angle_unified += 2 * M_PI;
    }
    return angle_unified;
}


bool PathPublisher::isLegal(const double x) {
    return (!std::isnan(x)) && (!std::isinf(x));
}


} // namespace me5413