#ifndef PATH_GENERATOR_HPP_
#define PATH_GENERATOR_HPP_

#include <ros/ros.h>
#include <vector>
#include <cmath>

namespace pure_pursuit_controller {

enum class PathType {
    CIRCLE,
    FIGURE8
};

struct PathPoint {
    double x;
    double y;
    double yaw;
};

class PathGenerator {
public:
    PathGenerator()  = default;
    ~PathGenerator() = default;

    std::vector<PathPoint> CreatePath(PathType type);

private:
    std::vector<PathPoint> CreateCircle();
    std::vector<PathPoint> CreateFigure8();
};

}  // namespace pure_pursuit_controller

#endif
