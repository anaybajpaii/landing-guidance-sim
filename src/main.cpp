#include <iostream>
#include "params.hpp"

int main() {
    Params p;
    std::cout << "gravity = " << p.gravity << " m/s^2\n";
    std::cout << "start mass = " << p.dry_mass + p.fuel_mass << " kg\n";
    std::cout << "dt = " << p.dt << " s\n";
    return 0;
}