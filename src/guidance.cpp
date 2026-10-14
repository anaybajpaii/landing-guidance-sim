#include "guidance.hpp"
#include <algorithm>
#include <cmath>

ConstantDecel::ConstantDecel(const Params& params) : params_(params) {}

double ConstantDecel::desiredVy(const LanderState& s, double) const {
    const double h = std::max(0.0, s.y);
    const double v_td = params_.touchdown_speed;
    return -std::sqrt(v_td * v_td + 2.0 * params_.const_decel * h);
}

LateBurn::LateBurn(const Params& params) : params_(params) {}

double LateBurn::desiredVy(const LanderState& s, double mass) const {
    const double h = std::max(0.0, s.y);
    const double a_max = params_.max_thrust / mass - params_.gravity;
    const double a = params_.burn_safety * a_max;
    const double v_td = params_.touchdown_speed;
    const double curve = -std::sqrt(v_td * v_td + 2.0 * a * h);

    if (s.vy > curve) {
        return s.vy - 100.0;
    }
    return curve;
}