#include "controller.hpp"
#include <algorithm>

Controller::Controller(const Params& params)
    : params_(params),
      vertical_(params.kp_vert, params.ki_vert, params.kd_vert,
                -params.gravity,
                params.max_thrust / params.dry_mass - params.gravity) {}

double Controller::throttle(const LanderState& s, double mass, double desired_vy) {
    const double error = desired_vy - s.vy;
    const double accel_cmd = vertical_.update(error, params_.dt);

    const double thrust = mass * (params_.gravity + accel_cmd);
    const double needed = thrust / params_.max_thrust;

    if (needed < 0.5 * params_.min_throttle) {
        return 0.0;
    }
    return std::clamp(needed, params_.min_throttle, 1.0);
}