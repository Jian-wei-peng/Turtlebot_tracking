#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include "pure_pursuit_controller/util/path_generator.hpp"

namespace pure_pursuit_controller {

class Visualizer {
public:
    Visualizer()  = default;
    ~Visualizer() = default;
    static visualization_msgs::Marker ConvertToMarker(const std::vector<PathPoint>& path_points);
    static nav_msgs::Path ConvertToROSPath(const std::vector<PathPoint>& path_points);

};

}  // namespace pure_pursuit_controller

#endif
