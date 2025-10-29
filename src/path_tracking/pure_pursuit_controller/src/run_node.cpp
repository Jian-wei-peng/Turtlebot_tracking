#include <ros/ros.h>
#include "pure_pursuit_controller/util/path_generator.hpp"
#include "pure_pursuit_controller/util/visualizer.hpp"
#include "pure_pursuit_controller/pure_pursuit.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller_node");
    ros::NodeHandle nh;

    // === publishers for visualization ===
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 1, true);
    ros::Publisher path_pub   = nh.advertise<nav_msgs::Path>("path", 1, true);

    // === generate path ===
    pure_pursuit_controller::PathGenerator path_generator;
    auto path_points = path_generator.CreatePath(pure_pursuit_controller::PathType::FIGURE8);

    // === prepare visual messages ===
    auto marker   = pure_pursuit_controller::Visualizer::ConvertToMarker(path_points);
    auto path_msg = pure_pursuit_controller::Visualizer::ConvertToROSPath(path_points);

    // === controller ===
    pure_pursuit_controller::PurePursuit controller(nh);

    ros::Rate rate(10);
    while (ros::ok()) {
        // publish visualization periodically (latched publishers used in node init;
        // we still publish here to keep stamp updated / make sure RViz receives)
        marker_pub.publish(marker);
        path_pub.publish(path_msg);

        // call single-step controller to compute & publish cmd_vel
        controller.ComputeAndPublish(path_points);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
