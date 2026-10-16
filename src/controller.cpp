#include "controller.hpp"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Controller::Controller(const Params& params)
    : params_(params),
      vertical_(params.kp_vert, params.ki_vert, params.kd_vert,
                -params.gravity,
                params.max_thrust / params.dry_mass - params.gravity),
      horizontal_(params.kp_horiz, params.ki_horiz, params.kd_horiz,
                  -params.max_thrust / params.dry_mass * std::sin(params.max_tilt_deg * kPi / 180.0),
                  params.max_thrust / params.dry_mass * std::sin(params.max_tilt_deg * kPi / 180.0)) {}

Command Controller::command(const LanderState& s, double mass, double desired_vy) {
    const double max_tilt = params_.max_tilt_deg * kPi / 180.0;

    const double vy_error = desired_vy - s.vy;
    const double accel_cmd = vertical_.update(vy_error, params_.dt);
    const double lift = std::max(params_.gravity + accel_cmd, 0.0);

    const double desired_vx = std::clamp(-params_.kp_pos * s.x,
                                         -params_.max_vx_cmd, params_.max_vx_cmd);
    const double ax_cmd = horizontal_.update(desired_vx - s.vx, params_.dt);
    const double tilt = std::clamp(std::atan2(ax_cmd, lift), -max_tilt, max_tilt);

    const double thrust = mass * lift / std::cos(tilt);
    const double needed = thrust / params_.max_thrust;

    Command cmd;
    if (needed < 0.5 * params_.min_throttle) {
        vertical_.reset();
        horizontal_.reset();
        return cmd;
    }
    cmd.throttle = std::clamp(needed, params_.min_throttle, 1.0);
    cmd.tilt = tilt;
    return cmd;
}