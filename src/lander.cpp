#include "lander.hpp"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Lander::Lander(const Params& params) : params_(params) {
    state_.t = 0.0;
    state_.x = params_.init_x;
    state_.y = params_.init_altitude;
    state_.vx = params_.init_vx;
    state_.vy = params_.init_vy;
    state_.fuel = params_.fuel_mass;
}

const LanderState& Lander::state() const {
    return state_;
}

double Lander::mass() const {
    return params_.dry_mass + state_.fuel;
}

bool Lander::hasFuel() const {
    return state_.fuel > 0.0;
}

bool Lander::onGround() const {
    return state_.y <= 0.0;
}

void Lander::step(double throttle_cmd, double tilt_cmd) {
    const double dt = params_.dt;
    const double m = mass();

    double throttle = 0.0;
    if (throttle_cmd > 0.0 && hasFuel()) {
        throttle = std::clamp(throttle_cmd, params_.min_throttle, 1.0);
    }

    const double max_tilt = params_.max_tilt_deg * kPi / 180.0;
    const double tilt = std::clamp(tilt_cmd, -max_tilt, max_tilt);

    const double thrust = throttle * params_.max_thrust;
    const double ax = thrust / m * std::sin(tilt);
    const double ay = thrust / m * std::cos(tilt) - params_.gravity;

    state_.vx += ax * dt;
    state_.vy += ay * dt;
    state_.x += state_.vx * dt;
    state_.y += state_.vy * dt;

    state_.fuel = std::max(0.0, state_.fuel - params_.mdot_max * throttle * dt);
    state_.t += dt;
    state_.thrust = thrust;
    state_.tilt = tilt;
}