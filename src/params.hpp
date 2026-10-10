#pragma once
#include <string>

enum class GuidanceType {
    ConstantDecel,
    LateBurn
};

struct Params {
    double gravity = 1.62;

    double dry_mass = 1000.0;
    double fuel_mass = 300.0;
    double max_thrust = 6000.0;
    double min_throttle = 0.2;
    double mdot_max = 2.0;
    double max_tilt_deg = 20.0;

    double init_altitude = 500.0;
    double init_x = 100.0;
    double init_vy = -40.0;
    double init_vx = 0.0;

    GuidanceType guidance = GuidanceType::ConstantDecel;
    double touchdown_speed = 1.0;
    double const_decel = 1.0;
    double burn_safety = 0.9;

    double kp_vert = 2.0;
    double ki_vert = 0.2;
    double kd_vert = 0.1;

    double kp_pos = 0.1;
    double max_vx_cmd = 5.0;
    double kp_horiz = 1.0;
    double ki_horiz = 0.0;
    double kd_horiz = 0.0;

    double max_land_speed = 2.0;
    double max_land_vx = 1.0;
    double max_land_dx = 5.0;

    double dt = 0.01;
    double max_time = 300.0;
    std::string output_file = "data/run.csv";
};