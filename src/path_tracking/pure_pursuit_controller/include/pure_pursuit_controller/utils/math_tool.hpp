#ifndef MATH_TOOL_HPP_
#define MATH_TOOL_HPP_

#include <vector>
#include <limits>
#include <cmath>
#include "pure_pursuit_controller/utils/path_generator.hpp"

namespace pure_pursuit_controller {

class MathTool {
public:
    MathTool() = default;
    ~MathTool() = default;

    double GetDistance(const PathPoint& p1, const PathPoint& p2);
    int GetMinDisIndex(const PathPoint& current_pose, const std::vector<PathPoint>& path);
};

}  // namespace
#endif
