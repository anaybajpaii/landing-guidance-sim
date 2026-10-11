#include <iomanip>
#include <iostream>
#include "params.hpp"
#include "lander.hpp"

int main() {
    Params p;
    Lander lander(p);

    while (!lander.onGround() && lander.state().t < p.max_time) {
        lander.step(0.0, 0.0);
    }

    const LanderState& s = lander.state();

    std::cout << std::fixed << std::setprecision(2);
    if (lander.onGround()) {
        std::cout << "Touchdown\n";
    } else {
        std::cout << "Timed out before touchdown\n";
    }
    std::cout << "time:             " << s.t << " s\n";
    std::cout << "altitude:         " << s.y << " m\n";
    std::cout << "vertical speed:   " << s.vy << " m/s\n";
    std::cout << "horizontal speed: " << s.vx << " m/s\n";
    std::cout << "horizontal error: " << s.x << " m\n";
    std::cout << "fuel used:        " << p.fuel_mass - s.fuel << " kg\n";

    return 0;
}