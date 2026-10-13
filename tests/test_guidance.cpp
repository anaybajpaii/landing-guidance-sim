#include <cmath>
#include <iostream>
#include "guidance.hpp"
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

LanderState at(double altitude) {
    LanderState s;
    s.y = altitude;
    return s;
}

void testEndpoints() {
    Params p;
    ConstantDecel law(p);
    check(near(law.desiredVy(at(0.0), 1300.0), -p.touchdown_speed, 1e-12),
          "constant decel: at the ground the target is -touchdown_speed");

    Params q;
    q.touchdown_speed = 0.0;
    q.const_decel = 2.0;
    ConstantDecel hand(q);
    check(near(hand.desiredVy(at(100.0), 1300.0), -20.0, 1e-9),
          "constant decel: a=2, h=100, v_td=0 gives -20 m/s by hand");
}

void testCurveShape() {
    Params p;
    ConstantDecel law(p);

    const double v = law.desiredVy(at(500.0), 1300.0);
    check(near(v * v, p.touchdown_speed * p.touchdown_speed + 2.0 * p.const_decel * 500.0, 1e-9),
          "constant decel: v^2 = v_td^2 + 2*a*h holds at 500 m");

    bool always_negative = true;
    bool increasing = true;
    double previous = law.desiredVy(at(0.0), 1300.0);
    for (double h = 10.0; h <= 500.0; h += 10.0) {
        const double current = law.desiredVy(at(h), 1300.0);
        if (current >= 0.0) {
            always_negative = false;
        }
        if (current >= previous) {
            increasing = false;
        }
        previous = current;
    }
    check(always_negative, "constant decel: target is always a descent");
    check(increasing, "constant decel: target gets faster as altitude increases");
}

void testInputs() {
    Params p;
    ConstantDecel law(p);

    check(near(law.desiredVy(at(-0.3), 1300.0), law.desiredVy(at(0.0), 1300.0), 1e-12),
          "constant decel: altitude below ground is treated as zero");
    check(near(law.desiredVy(at(200.0), 1300.0), law.desiredVy(at(200.0), 900.0), 1e-12),
          "constant decel: target does not depend on mass");

    Params q;
    q.const_decel = 2.0;
    ConstantDecel stronger(q);
    check(stronger.desiredVy(at(100.0), 1300.0) < law.desiredVy(at(100.0), 1300.0),
          "constant decel: larger a gives a faster target at the same height");
}

void testThroughBase() {
    Params p;
    ConstantDecel law(p);
    const Guidance& base = law;
    check(near(base.desiredVy(at(250.0), 1300.0), law.desiredVy(at(250.0), 1300.0), 1e-12),
          "interface: calling through the Guidance base gives the same answer");
}

}

int main() {
    testEndpoints();
    testCurveShape();
    testInputs();
    testThroughBase();

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}