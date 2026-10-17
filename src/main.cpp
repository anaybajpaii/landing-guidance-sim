#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "params.hpp"
#include "config.hpp"
#include "lander.hpp"
#include "logger.hpp"
#include "guidance.hpp"
#include "controller.hpp"
#include "touchdown.hpp"

int main(int argc, char** argv) {
    Params p;
    bool show_help = false;
    std::string error;
    const std::vector<std::string> args(argv + 1, argv + argc);
    if (!parseArgs(args, p, show_help, error)) {
        std::cerr << "Error: " << error << "\nRun with --help to see the settings.\n";
        return 2;
    }
    if (show_help) {
        std::cout << usageText();
        return 0;
    }

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
                  << " (does the folder exist?)\n";
        return 1;
    }

    LanderState previous = lander.state();
    logger.log(lander.state(), lander.mass());
    while (!lander.onGround() && lander.state().t < p.max_time) {
        const double desired_vy = guidance->desiredVy(lander.state(), lander.mass());
        const Command cmd = controller.command(lander.state(), lander.mass(), desired_vy);
        previous = lander.state();
        lander.step(cmd.throttle, cmd.tilt);
        logger.log(lander.state(), lander.mass());
    }

    const TouchdownResult r = evaluateTouchdown(previous, lander.state(), p);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "guidance: " << (p.guidance == GuidanceType::LateBurn ? "late-burn" : "constant-decel")
              << ", gravity " << p.gravity << " m/s^2\n";
    if (!r.landed) {
        std::cout << "Result: FAIL (did not touch down before max_time)\n";
    } else if (r.success) {
        std::cout << "Result: SUCCESS\n";
    } else {
        std::cout << "Result: FAIL\n";
    }
    std::cout << "time:           " << r.time << " s\n";
    if (r.landed) {
        std::cout << "landing speed:  " << r.vertical_speed << " m/s (limit "
                  << p.max_land_speed << ")" << (r.speed_ok ? "" : "  <-- too fast") << "\n";
        std::cout << "sideways speed: " << r.horizontal_speed << " m/s (limit "
                  << p.max_land_vx << ")" << (r.sideways_ok ? "" : "  <-- too fast") << "\n";
        std::cout << "miss distance:  " << r.miss_distance << " m (limit "
                  << p.max_land_dx << ")" << (r.position_ok ? "" : "  <-- too far") << "\n";
    }
    std::cout << "fuel used:      " << r.fuel_used << " kg\n";
    std::cout << "log written to: " << p.output_file << "\n";

    return 0;
}