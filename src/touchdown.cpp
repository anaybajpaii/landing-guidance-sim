#include "touchdown.hpp"
#include <cmath>

TouchdownResult evaluateTouchdown(const LanderState& previous,
                                  const LanderState& current,
                                  const Params& params) {
    TouchdownResult r;
    r.landed = current.y <= 0.0;

    LanderState at_ground = current;
    if (r.landed && previous.y > 0.0) {
        const double f = previous.y / (previous.y - current.y);
        at_ground.t = previous.t + f * (current.t - previous.t);
        at_ground.x = previous.x + f * (current.x - previous.x);
        at_ground.vx = previous.vx + f * (current.vx - previous.vx);
        at_ground.vy = previous.vy + f * (current.vy - previous.vy);
        at_ground.y = 0.0;
    }

    r.time = at_ground.t;
    r.vertical_speed = std::fabs(at_ground.vy);
    r.horizontal_speed = std::fabs(at_ground.vx);
    r.miss_distance = std::fabs(at_ground.x);
    r.fuel_used = params.fuel_mass - current.fuel;

    r.speed_ok = r.vertical_speed <= params.max_land_speed;
    r.sideways_ok = r.horizontal_speed <= params.max_land_vx;
    r.position_ok = r.miss_distance <= params.max_land_dx;
    r.success = r.landed && r.speed_ok && r.sideways_ok && r.position_ok;
    return r;
}