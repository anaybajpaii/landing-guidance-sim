#pragma once
#include "lander.hpp"
#include "params.hpp"
#include "pid.hpp"

class Controller {
public:
    explicit Controller(const Params& params);

    double throttle(const LanderState& s, double mass, double desired_vy);

private:
    Params params_;
    PID vertical_;
};