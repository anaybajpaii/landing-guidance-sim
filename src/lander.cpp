#include "lander.hpp"

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