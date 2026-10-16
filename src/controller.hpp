#pragma once
#include "lander.hpp"
#include "params.hpp"
#include "pid.hpp"

struct Command {
    double throttle = 0.0;
    double tilt = 0.0;
};

class Controller {
public:
    explicit Controller(const Params& params);

    Command command(const LanderState& s, double mass, double desired_vy);

private:
    Params params_;
    PID vertical_;
    PID horizontal_;
};