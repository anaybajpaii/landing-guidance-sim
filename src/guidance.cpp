#include "guidance.hpp"
#include <algorithm>
#include <cmath>

ConstantDecel::ConstantDecel(const Params& params) : params_(params) {}

double ConstantDecel::desiredVy(const LanderState& s, double) const {
    const double h = std::max(0.0, s.y);
    const double v_td = params_.touchdown_speed;
    return -std::sqrt(v_td * v_td + 2.0 * params_.const_decel * h);
}