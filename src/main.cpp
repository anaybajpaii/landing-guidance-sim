#include <iomanip>
#include <iostream>
#include <memory>
#include "params.hpp"
#include "lander.hpp"
#include "logger.hpp"
#include "guidance.hpp"
#include "controller.hpp"

int main() {
    Params p;
    Lander lander(p);
    std::unique_ptr<Guidance> guidance;
    if (p.guidance == GuidanceType::LateBurn) {
        guidance = std::make_unique<LateBurn>(p);
    } else {
        guidance = std::make_unique<ConstantDecel>(p);
    }
    Controller controller(p);

    DataLogger logger(p.output_file);
    if (!logger.isOpen()) {
        std::cerr << "Could not open " << p.output_file
                  << " (does the data folder exist?)\n";
        return 1;
    }

    logger.log(lander.state(), lander.mass());
    while (!lander.onGround() && lander.state().t < p.max_time) {
        const double desired_vy = guidance->desiredVy(lander.state(), lander.mass());
        const double throttle = controller.throttle(lander.state(), lander.mass(), desired_vy);
        lander.step(throttle, 0.0);
        logger.log(lander.state(), lander.mass());
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
    std::cout << "log written to:   " << p.output_file << "\n";

    return 0;
}