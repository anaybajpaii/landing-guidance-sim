#include <cmath>
#include <iostream>
#include "pid.hpp"

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

void testProportional() {
    PID pid(2.0, 0.0, 0.0, -100.0, 100.0);
    check(near(pid.update(3.0, 0.1), 6.0, 1e-12), "P: output is kp times error");
    check(near(pid.update(-1.5, 0.1), -3.0, 1e-12), "P: negative error gives negative output");
}

void testIntegral() {
    PID pid(0.0, 1.0, 0.0, -100.0, 100.0);
    check(near(pid.update(2.0, 0.5), 1.0, 1e-12), "I: first update adds ki*error*dt");
    check(near(pid.update(2.0, 0.5), 2.0, 1e-12), "I: second update accumulates");
}

void testDerivative() {
    PID pid(0.0, 0.0, 1.0, -100.0, 100.0);
    check(near(pid.update(1.0, 0.1), 0.0, 1e-12), "D: first update has no derivative");
    check(near(pid.update(2.0, 0.1), 10.0, 1e-9), "D: second update is kd times change over dt");
}

void testOutputClamp() {
    PID pid(10.0, 0.0, 0.0, -5.0, 5.0);
    check(near(pid.update(1.0, 0.1), 5.0, 1e-12), "clamp: output is capped at the maximum");
    check(near(pid.update(-1.0, 0.1), -5.0, 1e-12), "clamp: output is capped at the minimum");
}

void testAntiWindup() {
    PID pid(0.0, 1.0, 0.0, -1.0, 1.0);
    double out = 0.0;
    for (int i = 0; i < 10; ++i) {
        out = pid.update(10.0, 1.0);
    }
    check(near(out, 1.0, 1e-12), "windup: output saturates at the limit");
    out = pid.update(-0.5, 1.0);
    check(near(out, 0.5, 1e-12), "windup: integral recovers at once instead of staying wound up");
}

void testReset() {
    PID pid(0.0, 1.0, 1.0, -100.0, 100.0);
    pid.update(2.0, 1.0);
    pid.update(3.0, 1.0);
    pid.reset();
    check(near(pid.update(0.0, 1.0), 0.0, 1e-12), "reset: integral and derivative history are cleared");
}

void testZeroDt() {
    PID pid(2.0, 1.0, 1.0, -100.0, 100.0);
    check(near(pid.update(1.0, 0.0), 2.0, 1e-12), "dt zero: only the P term acts");
    check(near(pid.update(1.0, 0.0), 2.0, 1e-12), "dt zero: nothing accumulates");
}

}

int main() {
    testProportional();
    testIntegral();
    testDerivative();
    testOutputClamp();
    testAntiWindup();
    testReset();
    testZeroDt();

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}