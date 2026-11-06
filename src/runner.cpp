#include "runner.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <memory>
#include "config.hpp"
#include "controller.hpp"
#include "guidance.hpp"
#include "lander.hpp"
#include "logger.hpp"

namespace {

std::string trim(const std::string& text) {
    const char* spaces = " \t\r\n";
    const std::size_t first = text.find_first_not_of(spaces);
    if (first == std::string::npos) {
        return "";
    }
    const std::size_t last = text.find_last_not_of(spaces);
    return text.substr(first, last - first + 1);
}

std::string csvField(const std::string& text) {
    if (text.find_first_of(",\"\n") == std::string::npos) {
        return text;
    }
    std::string quoted = "\"";
    for (char c : text) {
        if (c == '"') {
            quoted += "\"\"";
        } else {
            quoted += c;
        }
    }
    quoted += "\"";
    return quoted;
}

const char* verdict(const TouchdownResult& r) {
    if (!r.landed) {
        return "TIMEOUT";
    }
    return r.success ? "SUCCESS" : "FAIL";
}

void tableCell(std::ostream& out, double value, bool show, int width) {
    if (show) {
        out << std::setw(width) << value;
    } else {
        out << std::setw(width) << "-";
    }
}

}

const char* guidanceName(GuidanceType type) {
    return type == GuidanceType::LateBurn ? "late-burn" : "constant-decel";
}

std::string numberedFile(const std::string& path, int index) {
    const std::string suffix = "_" + std::to_string(index);
    const std::size_t slash = path.find_last_of("/\\");
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
        return path + suffix;
    }
    return path.substr(0, dot) + suffix + path.substr(dot);
}

bool buildRuns(const Params& base, const std::vector<std::string>& specs,
               std::vector<RunPlan>& plan, std::string& error) {
    plan.clear();
    for (std::size_t i = 0; i < specs.size(); ++i) {
        RunPlan run;
        run.label = trim(specs[i]);
        if (run.label.empty()) {
            run.label = "base";
        }
        run.params = base;

        const std::string where = "run " + std::to_string(i + 1) + " (" + run.label + "): ";
        std::string problem;
        if (!applyRunSpec(run.params, specs[i], problem)) {
            error = where + problem;
            return false;
        }
        if (run.params.output_file != base.output_file || run.params.summary_file != base.summary_file) {
            error = where + "output_file and summary_file cannot be set inside --run";
            return false;
        }

        run.params.output_file = numberedFile(base.output_file, static_cast<int>(i) + 1);
        plan.push_back(run);
    }
    return true;
}

bool runScenario(const Params& p, TouchdownResult& result, std::string& error) {
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
        error = "Could not open " + p.output_file + " (does the folder exist?)";
        return false;
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

    result = evaluateTouchdown(previous, lander.state(), p);
    return true;
}

bool writeSummary(const std::string& path, const std::vector<RunResult>& results, std::string& error) {
    std::ofstream file(path);
    if (!file.is_open()) {
        error = "Could not open " + path + " (does the folder exist?)";
        return false;
    }

    file << "run,label,guidance,kp_vert,ki_vert,kd_vert,landed,success,"
            "time,landing_speed,sideways_speed,miss_distance,fuel_used,csv_file\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        const RunResult& r = results[i];
        const TouchdownResult& t = r.touchdown;
        file << i + 1 << ','
             << csvField(r.label) << ','
             << guidanceName(r.params.guidance) << ','
             << r.params.kp_vert << ','
             << r.params.ki_vert << ','
             << r.params.kd_vert << ','
             << (t.landed ? 1 : 0) << ','
             << (t.success ? 1 : 0) << ','
             << t.time << ','
             << t.vertical_speed << ','
             << t.horizontal_speed << ','
             << t.miss_distance << ','
             << t.fuel_used << ','
             << csvField(r.params.output_file) << '\n';
    }
    return true;
}

void printResult(std::ostream& out, const Params& p, const TouchdownResult& r) {
    out << std::fixed << std::setprecision(2);
    out << "guidance: " << guidanceName(p.guidance) << ", gravity " << p.gravity << " m/s^2\n";
    if (!r.landed) {
        out << "Result: FAIL (did not touch down before max_time)\n";
    } else if (r.success) {
        out << "Result: SUCCESS\n";
    } else {
        out << "Result: FAIL\n";
    }
    out << "time:           " << r.time << " s\n";
    if (r.landed) {
        out << "landing speed:  " << r.vertical_speed << " m/s (limit "
            << p.max_land_speed << ")" << (r.speed_ok ? "" : "  <-- too fast") << "\n";
        out << "sideways speed: " << r.horizontal_speed << " m/s (limit "
            << p.max_land_vx << ")" << (r.sideways_ok ? "" : "  <-- too fast") << "\n";
        out << "miss distance:  " << r.miss_distance << " m (limit "
            << p.max_land_dx << ")" << (r.position_ok ? "" : "  <-- too far") << "\n";
    }
    out << "fuel used:      " << r.fuel_used << " kg\n";
    out << "log written to: " << p.output_file << "\n";
}

void printTable(std::ostream& out, const std::vector<RunResult>& results) {
    std::size_t width = 5;
    for (const RunResult& r : results) {
        width = std::max(width, r.label.size());
    }

    out << std::fixed << std::setprecision(2);
    out << std::left << std::setw(4) << "run"
        << std::setw(static_cast<int>(width) + 2) << "label"
        << std::setw(9) << "result"
        << std::right
        << std::setw(9) << "time(s)"
        << std::setw(14) << "landing(m/s)"
        << std::setw(15) << "sideways(m/s)"
        << std::setw(10) << "miss(m)"
        << std::setw(11) << "fuel(kg)" << "\n";

    for (std::size_t i = 0; i < results.size(); ++i) {
        const RunResult& r = results[i];
        const TouchdownResult& t = r.touchdown;
        out << std::left << std::setw(4) << i + 1
            << std::setw(static_cast<int>(width) + 2) << r.label
            << std::setw(9) << verdict(t)
            << std::right;
        tableCell(out, t.time, true, 9);
        tableCell(out, t.vertical_speed, t.landed, 14);
        tableCell(out, t.horizontal_speed, t.landed, 15);
        tableCell(out, t.miss_distance, t.landed, 10);
        tableCell(out, t.fuel_used, true, 11);
        out << "\n";
    }
}