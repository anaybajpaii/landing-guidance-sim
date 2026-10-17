#include <cmath>
#include <iostream>
#include "lander.hpp"
#include "params.hpp"
#include "touchdown.hpp"

namespace {

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

LanderState state(double t, double x, double y, double vx, double vy, double fuel) {
    LanderState s;
    s.t = t;
    s.x = x;
    s.y = y;
    s.vx = vx;
    s.vy = vy;
    s.fuel = fuel;
    return s;
}

void testInterpolation() {
    Params p;
    const LanderState prev = state(2.0, 4.0, 3.0, 0.0, -4.0, 270.0);
    const LanderState cur = state(2.04, 0.0, -1.0, 0.4, 0.0, 269.0);
    const TouchdownResult r = evaluateTouchdown(prev, cur, p);

    check(r.landed, "interpolation: crossing the ground counts as landed");
    check(near(r.time, 2.03, 1e-12), "interpolation: time is the moment of ground contact");
    check(near(r.vertical_speed, 1.0, 1e-12), "interpolation: vertical speed at ground contact");
    check(near(r.horizontal_speed, 0.3, 1e-12), "interpolation: sideways speed at ground contact");
    check(near(r.miss_distance, 1.0, 1e-12), "interpolation: miss distance at ground contact");
}

void testSuccess() {
    Params p;
    const LanderState prev = state(30.0, 0.5, 0.02, 0.2, -0.9, 262.0);
    const LanderState cur = state(30.01, 0.5, -0.01, 0.2, -0.9, 261.18);
    const TouchdownResult r = evaluateTouchdown(prev, cur, p);

    check(r.success, "success: soft, slow and on the pad");
    check(r.speed_ok && r.sideways_ok && r.position_ok, "success: every limit is met");
    check(near(r.fuel_used, 38.82, 1e-9), "success: fuel used is starting fuel minus remaining");
}

void testEachFailure() {
    Params p;

    {
        const LanderState prev = state(10.0, 0.0, 0.5, 0.0, -9.0, 250.0);
        const LanderState cur = state(10.01, 0.0, -0.5, 0.0, -9.0, 250.0);
        const TouchdownResult r = evaluateTouchdown(prev, cur, p);
        check(!r.success && !r.speed_ok && r.sideways_ok && r.position_ok,
              "failure: crash speed fails only the speed check");
    }
    {
        const LanderState prev = state(10.0, 0.0, 0.5, 4.0, -1.0, 250.0);
        const LanderState cur = state(10.01, 0.0, -0.5, 4.0, -1.0, 250.0);
        const TouchdownResult r = evaluateTouchdown(prev, cur, p);
        check(!r.success && r.speed_ok && !r.sideways_ok && r.position_ok,
              "failure: sliding sideways fails only the sideways check");
    }
    {
        const LanderState prev = state(10.0, 30.0, 0.5, 0.0, -1.0, 250.0);
        const LanderState cur = state(10.01, 30.0, -0.5, 0.0, -1.0, 250.0);
        const TouchdownResult r = evaluateTouchdown(prev, cur, p);
        check(!r.success && r.speed_ok && r.sideways_ok && !r.position_ok,
              "failure: landing far from the pad fails only the position check");
    }
}

void testNotLanded() {
    Params p;
    const LanderState prev = state(299.99, 0.0, 12.0, 0.0, 0.0, 100.0);
    const LanderState cur = state(300.0, 0.0, 12.0, 0.0, 0.0, 100.0);
    const TouchdownResult r = evaluateTouchdown(prev, cur, p);

    check(!r.landed && !r.success, "not landed: still in the air is a failure");
    check(near(r.time, 300.0, 1e-12), "not landed: time is the final time");
}

void testBoundaries() {
    Params p;
    const LanderState ground = state(5.0, p.max_land_dx, 0.0, p.max_land_vx, -p.max_land_speed, 200.0);
    const TouchdownResult r = evaluateTouchdown(ground, ground, p);
    check(r.success, "limits: values exactly on a limit still count as success");
}

void testNegativeValues() {
    Params p;
    const LanderState prev = state(10.0, -2.0, 0.5, -0.5, -1.0, 250.0);
    const LanderState cur = state(10.01, -2.0, -0.5, -0.5, -1.0, 250.0);
    const TouchdownResult r = evaluateTouchdown(prev, cur, p);
    check(r.success && near(r.miss_distance, 2.0, 1e-12) && near(r.horizontal_speed, 0.5, 1e-12),
          "signs: landing left of the pad uses magnitudes");
}

}

int main() {
    testInterpolation();
    testSuccess();
    testEachFailure();
    testNotLanded();
    testBoundaries();
    testNegativeValues();

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}