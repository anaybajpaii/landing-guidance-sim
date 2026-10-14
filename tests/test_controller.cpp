#include <cmath>
#include <iostream>
#include "controller.hpp"
#include "params.hpp"

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

Params simpleGains() {
    Params p;
    p.kp_vert = 1.0;
    p.ki_vert = 0.0;
    p.kd_vert = 0.0;
    return p;
}

double throttleFor(const Params& p, double mass, double error) {
    Controller c(p);
    LanderState s;
    s.vy = 0.0;
    return c.throttle(s, mass, error);
}

void testBasics() {
    const Params p = simpleGains();
    check(near(throttleFor(p, 1000.0, 0.0), 0.27, 1e-9),
          "hover: zero error commands thrust equal to weight");
    check(near(throttleFor(p, 1000.0, 1.38), 0.5, 1e-9),
          "proportional: error of 1.38 m/s adds the matching thrust");
}

void testMass() {
    const Params p = simpleGains();
    check(near(throttleFor(p, 1300.0, 0.0), 0.351, 1e-9),
          "mass: hover throttle at 1300 kg is m*g/max_thrust");
    check(throttleFor(p, 1300.0, 0.0) > throttleFor(p, 1000.0, 0.0),
          "mass: a heavier lander needs more throttle for the same error");
}

void testSaturation() {
    const Params p = simpleGains();
    check(near(throttleFor(p, 1000.0, 10.0), 1.0, 1e-12),
          "limits: huge positive error saturates at full throttle");
    check(near(throttleFor(p, 1000.0, -10.0), 0.0, 1e-12),
          "limits: huge negative error turns the engine off");
}

void testMinThrottle() {
    const Params p = simpleGains();
    check(near(throttleFor(p, 1000.0, -1.32), 0.0, 1e-12),
          "min throttle: request below half of min_throttle means engine off");
    check(near(throttleFor(p, 1000.0, -0.72), p.min_throttle, 1e-12),
          "min throttle: request between half and full min is raised to min");
}

void testDirection() {
    Params p;
    LanderState s;

    {
        Controller c(p);
        s.vy = -40.0;
        check(near(c.throttle(s, 1300.0, -31.6), 1.0, 1e-12),
              "direction: falling much faster than the curve gives full throttle");
    }
    {
        Controller c(p);
        s.vy = -20.0;
        check(near(c.throttle(s, 1300.0, -31.6), 0.0, 1e-12),
              "direction: falling much slower than the curve turns the engine off");
    }
}

void testIntegral() {
    Params p;
    Controller c(p);
    LanderState s;
    s.vy = -10.0;

    const double first = c.throttle(s, 1300.0, -9.9);
    double later = first;
    for (int i = 0; i < 100; ++i) {
        later = c.throttle(s, 1300.0, -9.9);
    }
    check(later > first, "integral: a persistent small error slowly raises the throttle");
}

void testResetWhenOff() {
    Params p;
    Controller c(p);
    LanderState s;
    s.vy = -10.0;
    for (int i = 0; i < 500; ++i) {
        c.throttle(s, 1300.0, -50.0);
    }
    const double after_coast = c.throttle(s, 1300.0, -9.9);

    Controller fresh(p);
    const double from_fresh = fresh.throttle(s, 1300.0, -9.9);
    check(near(after_coast, from_fresh, 1e-12), "engine off clears the PID");
}

}

int main() {
    testBasics();
    testMass();
    testSaturation();
    testMinThrottle();
    testDirection();
    testIntegral();
    testResetWhenOff();

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}