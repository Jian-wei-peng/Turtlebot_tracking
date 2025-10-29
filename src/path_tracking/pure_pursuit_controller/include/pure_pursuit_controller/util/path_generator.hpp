#ifndef PATH_GENERATOR_HPP_
#define PATH_GENERATOR_HPP_

#include <ros/ros.h>
#include <vector>
#include <cmath>

namespace pure_pursuit_controller {

enum class PathType {
    LINE,
    CIRCLE,
    FIGURE8,
    SINE_WAVE
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

public:
    ros::Publisher path_pub_;

private:
    std::vector<PathPoint> CreateLine();
    std::vector<PathPoint> CreateCircle();
    std::vector<PathPoint> CreateFigure8();
    std::vector<PathPoint> CreateSineWave();
};

}  // namespace

#endif
