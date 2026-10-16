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
    return c.command(s, mass, error).throttle;
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
        check(near(c.command(s, 1300.0, -31.6).throttle, 1.0, 1e-12),
              "direction: falling much faster than the curve gives full throttle");
    }
    {
        Controller c(p);
        s.vy = -20.0;
        check(near(c.command(s, 1300.0, -31.6).throttle, 0.0, 1e-12),
              "direction: falling much slower than the curve turns the engine off");
    }
}

void testIntegral() {
    Params p;
    Controller c(p);
    LanderState s;
    s.vy = -10.0;

    const double first = c.command(s, 1300.0, -9.9).throttle;
    double later = first;
    for (int i = 0; i < 100; ++i) {
        later = c.command(s, 1300.0, -9.9).throttle;
    }
    check(later > first, "integral: a persistent small error slowly raises the throttle");
}

void testResetWhenOff() {
    Params p;
    Controller c(p);
    LanderState s;
    s.vy = -10.0;
    for (int i = 0; i < 500; ++i) {
        c.command(s, 1300.0, -50.0);
    }
    const double after_coast = c.command(s, 1300.0, -9.9).throttle;

    Controller fresh(p);
    const double from_fresh = fresh.command(s, 1300.0, -9.9).throttle;
    check(near(after_coast, from_fresh, 1e-12), "engine off clears the PID");
}

void testTiltDirection() {
    Params p;
    LanderState s;
    s.vy = -10.0;

    {
        Controller c(p);
        s.x = 50.0;
        check(c.command(s, 1300.0, -10.0).tilt < 0.0,
              "tilt: pad is toward -x, so the thrust leans toward -x");
    }
    {
        Controller c(p);
        s.x = -50.0;
        check(c.command(s, 1300.0, -10.0).tilt > 0.0,
              "tilt: pad is toward +x, so the thrust leans toward +x");
    }
    {
        Controller c(p);
        s.x = 0.0;
        check(near(c.command(s, 1300.0, -10.0).tilt, 0.0, 1e-12),
              "tilt: on the pad and not moving sideways means no tilt");
    }
}

void testTiltLimit() {
    const double pi = 3.14159265358979323846;
    const Params p = simpleGains();
    const double max_tilt = p.max_tilt_deg * pi / 180.0;

    Controller c(p);
    LanderState s;
    s.x = 1000.0;
    const Command cmd = c.command(s, 1000.0, 0.0);
    check(near(cmd.tilt, -max_tilt, 1e-9), "tilt: a huge offset is limited to max_tilt_deg");
    check(near(cmd.throttle, 0.27 / std::cos(max_tilt), 1e-9),
          "tilt: throttle is raised by 1/cos(tilt) to keep the vertical thrust");
}

void testEngineOffNoTilt() {
    Controller c(simpleGains());
    LanderState s;
    s.x = 1000.0;
    const Command cmd = c.command(s, 1000.0, -10.0);
    check(near(cmd.throttle, 0.0, 1e-12) && near(cmd.tilt, 0.0, 1e-12),
          "engine off: no throttle and no tilt");
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
    testTiltDirection();
    testTiltLimit();
    testEngineOffNoTilt();

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}