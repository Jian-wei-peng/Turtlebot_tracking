#include "pure_pursuit_controller/util/visualizer.hpp"

namespace pure_pursuit_controller {

// ----------------- Marker -----------------
visualization_msgs::Marker Visualizer::ConvertToMarker(const std::vector<PathPoint>& path_points)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "odom";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    for (const auto& pt : path_points) {
        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0;
        marker.points.push_back(p);
    }
    return marker;
}

// ----------------- ROS Path -----------------
nav_msgs::Path Visualizer::ConvertToROSPath(const std::vector<PathPoint>& path_points)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& pt : path_points) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        double cy = cos(pt.yaw * 0.5);
        double sy = sin(pt.yaw * 0.5);
        pose.pose.orientation.w = cy;
        pose.pose.orientation.z = sy;
        path_msg.poses.push_back(pose);
    }
    return path_msg;
}

}  // namespace
