#pragma once
#include "lander.hpp"
#include "params.hpp"

struct TouchdownResult {
    bool landed = false;
    bool speed_ok = false;
    bool sideways_ok = false;
    bool position_ok = false;
    bool success = false;
    double time = 0.0;
    double vertical_speed = 0.0;
    double horizontal_speed = 0.0;
    double miss_distance = 0.0;
    double fuel_used = 0.0;
};

TouchdownResult evaluateTouchdown
(
    const LanderState& previous,
    const LanderState& current,
    const Params& params
);