#include "pure_pursuit_controller/util/visualizer.hpp"

namespace pure_pursuit_controller {

// Convert PathPoint to Marker
visualization_msgs::Marker Visualizer::ConvertToMarker(const std::vector<PathPoint>& path_points)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.points.clear();
    marker.points.reserve(path_points.size());

    for (const auto& pt : path_points) {
        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0;
        marker.points.push_back(p);
    }
    return marker;
}

// Convert PathPoint to nav_msgs/Path
nav_msgs::Path Visualizer::ConvertToROSPath(const std::vector<PathPoint>& path_points)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = ros::Time::now();

    path_msg.poses.clear();
    path_msg.poses.reserve(path_points.size());

    for (const auto& pt : path_points) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pt.yaw); // roll, pitch, yaw
        pose.pose.orientation = tf2::toMsg(q);

        path_msg.poses.push_back(pose);
    }
    return path_msg;
}

}  // namespace pure_pursuit_controller
