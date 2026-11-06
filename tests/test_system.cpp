#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include "controller.hpp"
#include "guidance.hpp"
#include "lander.hpp"
#include "params.hpp"
#include "pid.hpp"
#include "touchdown.hpp"

namespace {

const double kPi = 3.14159265358979323846;

int passed = 0;
int failed = 0;

void check(bool condition, const char* name) {
    if (condition) {
        ++passed;
        std::cout << "PASS  " << name << "\n";
    } else {
        ++failed;
        std::cout << "FAIL  " << name << "\n";
    }
}

bool near(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

struct Flight {
    TouchdownResult result;
    LanderState final_state;
    double start_fuel = 0.0;
    double max_thrust_seen = 0.0;
    double max_tilt_seen = 0.0;
    double smallest_nonzero_thrust = 1e18;
    double lowest_fuel = 1e18;
    double highest_fuel_step_gain = 0.0;
    bool time_always_increased = true;
    bool altitude_positive_until_end = true;
};

Flight fly(const Params& p) {
    Lander lander(p);
    std::unique_ptr<Guidance> guidance;
    if (p.guidance == GuidanceType::LateBurn) {
        guidance = std::make_unique<LateBurn>(p);
    } else {
        guidance = std::make_unique<ConstantDecel>(p);
    }
    Controller controller(p);

    Flight f;
    f.start_fuel = lander.state().fuel;
    LanderState previous = lander.state();
    while (!lander.onGround() && lander.state().t < p.max_time) {
        const double desired_vy = guidance->desiredVy(lander.state(), lander.mass());
        const Command cmd = controller.command(lander.state(), lander.mass(), desired_vy);
        previous = lander.state();
        lander.step(cmd.throttle, cmd.tilt);

        const LanderState& s = lander.state();
        if (s.t <= previous.t) {
            f.time_always_increased = false;
        }
        if (!(s.y > 0.0 || lander.onGround())) {
            f.altitude_positive_until_end = false;
        }
        f.max_thrust_seen = std::fmax(f.max_thrust_seen, s.thrust);
        f.max_tilt_seen = std::fmax(f.max_tilt_seen, std::fabs(s.tilt));
        if (s.thrust > 0.0) {
            f.smallest_nonzero_thrust = std::fmin(f.smallest_nonzero_thrust, s.thrust);
        }
        f.lowest_fuel = std::fmin(f.lowest_fuel, s.fuel);
        f.highest_fuel_step_gain = std::fmax(f.highest_fuel_step_gain, s.fuel - previous.fuel);
    }
    f.final_state = lander.state();
    f.result = evaluateTouchdown(previous, lander.state(), p);
    return f;
}

void testInvariants() {
    const Params p;
    const Flight f = fly(p);

    check(f.result.landed, "invariants: the default flight reaches the ground");
    check(f.time_always_increased, "invariants: time moves forward on every step");
    check(f.altitude_positive_until_end, "invariants: altitude is above ground until the final step");
    check(f.highest_fuel_step_gain <= 0.0, "invariants: fuel never increases");
    check(f.lowest_fuel >= 0.0, "invariants: fuel never goes negative");
    check(near(f.result.fuel_used, f.start_fuel - f.final_state.fuel, 1e-9),
          "invariants: fuel used equals fuel lost");
    check(f.max_thrust_seen <= p.max_thrust + 1e-9, "invariants: thrust never exceeds the engine maximum");
    check(f.max_tilt_seen <= p.max_tilt_deg * kPi / 180.0 + 1e-9,
          "invariants: tilt never exceeds the tilt limit");
    check(f.smallest_nonzero_thrust >= p.min_throttle * p.max_thrust - 1e-9,
          "invariants: a running engine is never below minimum throttle");
}

void testInvariantsLateBurn() {
    Params p;
    p.guidance = GuidanceType::LateBurn;
    const Flight f = fly(p);

    check(f.result.landed && f.highest_fuel_step_gain <= 0.0 && f.lowest_fuel >= 0.0,
          "late burn invariants: lands, fuel only decreases and stays non-negative");
    check(f.max_thrust_seen <= p.max_thrust + 1e-9
              && f.smallest_nonzero_thrust >= p.min_throttle * p.max_thrust - 1e-9,
          "late burn invariants: thrust stays within engine limits");
}

void testNoFuel() {
    Params p;
    p.fuel_mass = 0.0;
    const Flight f = fly(p);

    const double expected = std::sqrt(p.init_vy * p.init_vy + 2.0 * p.gravity * p.init_altitude);
    check(f.result.landed && !f.result.success, "no fuel: the lander falls and crashes");
    check(near(f.result.vertical_speed, expected, 0.5), "no fuel: impact speed matches free fall");
    check(near(f.result.fuel_used, 0.0, 1e-12), "no fuel: no fuel is used");
}

void testTimeStep() {
    Params coarse;
    coarse.dt = 0.01;
    Params fine;
    fine.dt = 0.002;
    const Flight a = fly(coarse);
    const Flight b = fly(fine);

    check(a.result.success && b.result.success, "time step: both step sizes land successfully");
    check(near(a.result.time, b.result.time, 0.2), "time step: landing time barely depends on dt");
    check(near(a.result.vertical_speed, b.result.vertical_speed, 0.15),
          "time step: landing speed barely depends on dt");
    check(near(a.result.fuel_used, b.result.fuel_used, 0.5), "time step: fuel use barely depends on dt");
}

void testSymmetry() {
    Params right;
    right.init_x = 50.0;
    Params left;
    left.init_x = -50.0;
    const Flight a = fly(right);
    const Flight b = fly(left);

    check(near(a.result.time, b.result.time, 1e-9) && near(a.result.fuel_used, b.result.fuel_used, 1e-9),
          "symmetry: a mirrored start takes the same time and fuel");
    check(near(a.final_state.x, -b.final_state.x, 1e-9) && near(a.final_state.vx, -b.final_state.vx, 1e-9),
          "symmetry: the final position and sideways velocity mirror");
}

void testOnPad() {
    Params p;
    p.init_x = 0.0;
    const Flight f = fly(p);

    check(f.result.success && near(f.result.miss_distance, 0.0, 1e-6), "on pad: starting above the pad lands on it");
    check(near(f.max_tilt_seen, 0.0, 1e-9), "on pad: no tilt is needed, so none is used");
}

void testScenarios() {
    const double altitudes[] = {300.0, 500.0, 800.0};
    for (double altitude : altitudes) {
        Params p;
        p.init_altitude = altitude;
        const Flight f = fly(p);
        std::string name = "altitude sweep: constant decel lands from " + std::to_string(static_cast<int>(altitude)) + " m";
        check(f.result.success, name.c_str());
    }

    Params mars;
    mars.gravity = 3.71;
    mars.max_thrust = 9000.0;
    mars.mdot_max = 3.0;
    const Flight m = fly(mars);
    check(m.result.success, "scenario: the Mars settings land with constant decel");

    Params stronger;
    stronger.const_decel = 1.5;
    Params gentler;
    gentler.const_decel = 0.7;
    const Flight s = fly(stronger);
    const Flight g = fly(gentler);
    check(s.result.landed && g.result.landed && s.result.time < g.result.time,
          "scenario: a larger planned deceleration gives a shorter flight");
}

void testPid() {
    PID a(2.0, 0.0, 0.0, -100.0, 100.0);
    PID b(2.0, 0.0, 0.0, -100.0, 100.0);
    check(near(a.update(3.0, 0.1), -b.update(-3.0, 0.1), 1e-12), "pid: the response is odd-symmetric");

    PID zero(2.0, 1.0, 1.0, -100.0, 100.0);
    check(near(zero.update(0.0, 0.1), 0.0, 1e-12), "pid: zero error gives zero output on the first call");

    PID clamp(100.0, 0.0, 0.0, -5.0, 5.0);
    check(near(clamp.update(10.0, 0.1), 5.0, 1e-12) && near(clamp.update(-10.0, 0.1), -5.0, 1e-12),
          "pid: the output is clamped on both sides");

    PID integral(0.0, 1.0, 0.0, -100.0, 100.0);
    const double first = integral.update(2.0, 0.5);
    const double second = integral.update(2.0, 0.5);
    check(near(second, 2.0 * first, 1e-9), "pid: a steady error makes the integral grow linearly");

    integral.reset();
    check(near(integral.update(2.0, 0.5), first, 1e-12), "pid: reset forgets the accumulated integral");
}

void testGuidanceExtras() {
    const Params p;
    ConstantDecel law(p);
    LanderState s;

    double previous = 0.0;
    bool monotonic = true;
    for (int h = 0; h <= 1000; h += 50) {
        s.y = h;
        const double v = law.desiredVy(s, 1300.0);
        if (h > 0 && v >= previous) {
            monotonic = false;
        }
        previous = v;
    }
    check(monotonic, "guidance extra: the target descent speed grows steadily with altitude");

    LateBurn late(p);
    s.y = 200.0;
    s.vy = -10.0;
    check(late.desiredVy(s, 1300.0) < s.vy - 50.0,
          "guidance extra: late burn coasts while the lander is below the braking curve");
}

}

int main() {
    testInvariants();
    testInvariantsLateBurn();
    testNoFuel();
    testTimeStep();
    testSymmetry();
    testOnPad();
    testScenarios();
    testPid();
    testGuidanceExtras();

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}