#include "pure_pursuit_controller/util/path_generator.hpp"

namespace pure_pursuit_controller {

std::vector<PathPoint> PathGenerator::CreatePath(PathType type) {
    switch (type) {
        case PathType::CIRCLE:    return CreateCircle();
        case PathType::FIGURE8:   return CreateFigure8();
        default:                  return {};
    }
}

std::vector<PathPoint> PathGenerator::CreateCircle() {
    std::vector<PathPoint> path_vec;
    int slices = 100;
    double r = 2.0;
    for (int i = 0; i <= slices; i++) {
        double angle = i * 2 * M_PI / slices;
        path_vec.push_back({r * cos(angle), r * sin(angle), angle + M_PI/2});
    }
    return path_vec;
}

std::vector<PathPoint> PathGenerator::CreateFigure8() {
    std::vector<PathPoint> path_vec;
    int slices = 200;
    double a = 3.0;
    for (int i = 0; i <= slices; i++) {
        double t = i * 2 * M_PI / slices;
        double x = a * sin(t);
        double y = a * sin(t) * cos(t);
        double dx = a * cos(t);
        double dy = a * cos(2*t);
        double yaw = atan2(dy, dx);
        path_vec.push_back({x, y, yaw});
    }
    return path_vec;
}

}  // namespace pure_pursuit_controller
