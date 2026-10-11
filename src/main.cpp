#include <iostream>
#include "params.hpp"
#include "lander.hpp"

int main() {
    Params p;

    Lander freefall(p);
    for (int i = 0; i < 100; ++i) {
        freefall.step(0.0, 0.0);
    }
    const LanderState& a = freefall.state();
    std::cout << "free fall 1 s: t = " << a.t << ", y = " << a.y
              << ", vy = " << a.vy << ", fuel = " << a.fuel << "\n";

    Lander burning(p);
    for (int i = 0; i < 100; ++i) {
        burning.step(1.0, 0.0);
    }
    const LanderState& b = burning.state();
    std::cout << "full thrust 1 s: t = " << b.t << ", y = " << b.y
              << ", vy = " << b.vy << ", fuel = " << b.fuel << "\n";

    return 0;
}