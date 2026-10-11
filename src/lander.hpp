#pragma once
#include "params.hpp"

struct LanderState {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double fuel = 0.0;
    double thrust = 0.0;
    double tilt = 0.0;
};

class Lander {
public:
    explicit Lander(const Params& params);

    const LanderState& state() const;

    double mass() const;
    bool hasFuel() const;
    bool onGround() const;

    void step(double throttle_cmd, double tilt_cmd);

private:
    Params params_;
    LanderState state_;
};