#include "pid.hpp"
#include <algorithm>

PID::PID(double kp, double ki, double kd, double out_min, double out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

double PID::update(double error, double dt) {
    double d_term = 0.0;

    if (dt > 0.0) {
        i_term_ += ki_ * error * dt;
        i_term_ = std::clamp(i_term_, out_min_, out_max_);
        if (has_prev_) {
            d_term = kd_ * (error - prev_error_) / dt;
        }
    }

    prev_error_ = error;
    has_prev_ = true;

    const double output = kp_ * error + i_term_ + d_term;
    return std::clamp(output, out_min_, out_max_);
}

void PID::reset() {
    i_term_ = 0.0;
    prev_error_ = 0.0;
    has_prev_ = false;
}