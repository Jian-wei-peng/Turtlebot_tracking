#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include "pure_pursuit_controller/util/path_generator.hpp"

namespace pure_pursuit_controller {

class Visualizer {
public:
    Visualizer()  = default;
    ~Visualizer() = default;

    // 将 PathPoint 转换为 Marker
    static visualization_msgs::Marker ConvertToMarker(const std::vector<PathPoint>& path_points);

    // 将 PathPoint 转换为 nav_msgs/Path
    static nav_msgs::Path ConvertToROSPath(const std::vector<PathPoint>& path_points);

public:
    ros::Publisher marker_pub_;
};

}  // namespace

#endif
