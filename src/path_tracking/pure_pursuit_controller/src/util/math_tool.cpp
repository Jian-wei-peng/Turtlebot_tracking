#include "pure_pursuit_controller/util/math_tool.hpp"
#include <limits>

namespace pure_pursuit_controller {

double MathTool::GetDistance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

int MathTool::GetMinDisIndex(const PathPoint& current_pose, const std::vector<PathPoint>& path) {
    double min_dis = std::numeric_limits<double>::infinity();
    int index = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        double d = GetDistance(current_pose, path[i]);
        if (d < min_dis) {
            min_dis = d;
            index = static_cast<int>(i);
        }
    }
    return index;
}

}  // namespace pure_pursuit_controller
