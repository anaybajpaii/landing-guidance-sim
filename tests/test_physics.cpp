#include <cmath>
#include <iostream>
#include "lander.hpp"
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

void testFreeFall() {
    Params p;
    Lander lander(p);
    for (int i = 0; i < 100; ++i) {
        lander.step(0.0, 0.0);
    }
    const LanderState& s = lander.state();
    const double t = s.t;
    const double expected_vy = p.init_vy - p.gravity * t;
    const double expected_y = p.init_altitude + p.init_vy * t - 0.5 * p.gravity * t * t;

    check(near(t, 1.0, 1e-9), "free fall: time advances by dt each step");
    check(near(s.vy, expected_vy, 1e-9), "free fall: vy matches v0 - g*t");
    check(near(s.y, expected_y, 0.02), "free fall: y matches kinematics within Euler error");
    check(near(s.fuel, p.fuel_mass, 1e-12), "free fall: no fuel used");
    check(near(s.vx, 0.0, 1e-12), "free fall: no horizontal motion");
}

void testHover() {
    Params p;
    Lander lander(p);
    const double hover_throttle = lander.mass() * p.gravity / p.max_thrust;
    lander.step(hover_throttle, 0.0);
    check(near(lander.state().vy, p.init_vy, 1e-9), "hover: thrust equal to weight leaves vy unchanged");
}

void testFuelBurn() {
    Params p;
    Lander lander(p);
    for (int i = 0; i < 100; ++i) {
        lander.step(1.0, 0.0);
    }
    const LanderState& s = lander.state();
    check(near(s.fuel, p.fuel_mass - p.mdot_max * 1.0, 1e-9), "fuel: full throttle burns mdot_max per second");
    check(near(lander.mass(), p.dry_mass + s.fuel, 1e-12), "mass: always dry mass plus fuel");
}

void testFuelOut() {
    Params p;
    p.fuel_mass = 0.05;
    Lander lander(p);

    bool never_negative = true;
    for (int i = 0; i < 10; ++i) {
        lander.step(1.0, 0.0);
        if (lander.state().fuel < 0.0) {
            never_negative = false;
        }
    }

    check(never_negative, "fuel out: fuel never goes negative");
    check(near(lander.state().fuel, 0.0, 1e-12), "fuel out: tank ends empty");
    check(!lander.hasFuel(), "fuel out: hasFuel is false");
    check(near(lander.state().thrust, 0.0, 1e-12), "fuel out: no thrust once the tank is empty");
}

void testThrottleLimits() {
    Params p;

    Lander low(p);
    low.step(0.05, 0.0);
    check(near(low.state().thrust, p.min_throttle * p.max_thrust, 1e-9),
          "throttle: small command is raised to min_throttle");

    Lander off(p);
    off.step(0.0, 0.0);
    check(near(off.state().thrust, 0.0, 1e-12), "throttle: zero command means engine off");

    Lander high(p);
    high.step(5.0, 0.0);
    check(near(high.state().thrust, p.max_thrust, 1e-9), "throttle: large command is capped at max thrust");
}

void testTilt() {
    const double pi = 3.14159265358979323846;
    Params p;
    const double max_tilt = p.max_tilt_deg * pi / 180.0;

    Lander right(p);
    right.step(1.0, 1.0);
    check(near(right.state().tilt, max_tilt, 1e-9), "tilt: large positive command is clamped");
    check(right.state().vx > 0.0, "tilt: positive tilt accelerates toward +x");

    Lander left(p);
    left.step(1.0, -1.0);
    check(near(left.state().tilt, -max_tilt, 1e-9), "tilt: large negative command is clamped");
    check(left.state().vx < 0.0, "tilt: negative tilt accelerates toward -x");
}

void testOnGround() {
    Params p;
    Lander air(p);
    check(!air.onGround(), "ground: default start is in the air");

    p.init_altitude = 0.0;
    Lander ground(p);
    check(ground.onGround(), "ground: altitude zero counts as on the ground");
}

}

int main() {
    testFreeFall();
    testHover();
    testFuelBurn();
    testFuelOut();
    testThrottleLimits();
    testTilt();
    testOnGround();

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}