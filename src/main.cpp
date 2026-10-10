#include <iostream>
#include "params.hpp"
#include "lander.hpp"

int main() {
    Params p;
    Lander lander(p);
    const LanderState& s = lander.state();

    std::cout << std::boolalpha;
    std::cout << "t = " << s.t << " s\n";
    std::cout << "x = " << s.x << " m, y = " << s.y << " m\n";
    std::cout << "vx = " << s.vx << " m/s, vy = " << s.vy << " m/s\n";
    std::cout << "mass = " << lander.mass() << " kg\n";
    std::cout << "has fuel: " << lander.hasFuel() << "\n";
    std::cout << "on ground: " << lander.onGround() << "\n";
    return 0;
}