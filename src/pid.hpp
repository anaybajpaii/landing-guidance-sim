#pragma once

class PID {
public:
    PID(double kp, double ki, double kd, double out_min, double out_max);

    double update(double error, double dt);
    void reset();

private:
    double kp_, ki_, kd_;
    double out_min_, out_max_;
    double i_term_ = 0.0;
    double prev_error_ = 0.0;
    bool has_prev_ = false;
};