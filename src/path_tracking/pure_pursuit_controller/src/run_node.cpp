#include <ros/ros.h>
#include "pure_pursuit_controller/util/path_generator.hpp"
#include "pure_pursuit_controller/util/visualizer.hpp"
#include "pure_pursuit_controller/pure_pursuit.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "pure_pursuit_controller");
    ros::NodeHandle nh("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/path_marker", 1, true);
    ros::Publisher path_pub   = nh.advertise<nav_msgs::Path>("/path", 1, true);

    // Generate path
    std::string path_type_str;
    pure_pursuit_controller::PathType path_type;
    nh.param("path_type", path_type_str, std::string("circle"));
    if (path_type_str == "figure8")
        path_type = pure_pursuit_controller::PathType::FIGURE8;
    else if (path_type_str == "circle")
        path_type = pure_pursuit_controller::PathType::CIRCLE;
    else {
        ROS_ERROR("Unsupported path type: %s", path_type_str.c_str());
        return -1;
    }
    pure_pursuit_controller::PathGenerator path_generator;
    auto path_points = path_generator.CreatePath(path_type);


    // Path visualization
    auto marker   = pure_pursuit_controller::Visualizer::ConvertToMarker(path_points);
    auto path_msg = pure_pursuit_controller::Visualizer::ConvertToROSPath(path_points);

    // Define pure pursuit controller and initialize
    pure_pursuit_controller::PurePursuit controller(nh);

    ros::Rate rate(10);
    while (ros::ok()) {
        // Publish reference path
        marker_pub.publish(marker);
        path_pub.publish(path_msg);
        
        // Execute pure pursuit control and publish cmd_vel
        controller.ComputeAndPublish(path_points);
        
        // Handle odom callback
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
