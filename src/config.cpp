#include "config.hpp"
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <utility>

namespace {

using Field = double Params::*;

const std::map<std::string, Field>& numericSettings() {
    static const std::map<std::string, Field> settings = {
        {"gravity", &Params::gravity},
        {"dry_mass", &Params::dry_mass},
        {"fuel_mass", &Params::fuel_mass},
        {"max_thrust", &Params::max_thrust},
        {"min_throttle", &Params::min_throttle},
        {"mdot_max", &Params::mdot_max},
        {"max_tilt_deg", &Params::max_tilt_deg},
        {"init_altitude", &Params::init_altitude},
        {"init_x", &Params::init_x},
        {"init_vy", &Params::init_vy},
        {"init_vx", &Params::init_vx},
        {"touchdown_speed", &Params::touchdown_speed},
        {"const_decel", &Params::const_decel},
        {"burn_safety", &Params::burn_safety},
        {"kp_vert", &Params::kp_vert},
        {"ki_vert", &Params::ki_vert},
        {"kd_vert", &Params::kd_vert},
        {"kp_pos", &Params::kp_pos},
        {"max_vx_cmd", &Params::max_vx_cmd},
        {"kp_horiz", &Params::kp_horiz},
        {"ki_horiz", &Params::ki_horiz},
        {"kd_horiz", &Params::kd_horiz},
        {"max_land_speed", &Params::max_land_speed},
        {"max_land_vx", &Params::max_land_vx},
        {"max_land_dx", &Params::max_land_dx},
        {"dt", &Params::dt},
        {"max_time", &Params::max_time}
    };
    return settings;
}

std::string trim(const std::string& text) {
    const char* spaces = " \t\r\n";
    const std::size_t first = text.find_first_not_of(spaces);
    if (first == std::string::npos) {
        return "";
    }
    const std::size_t last = text.find_last_not_of(spaces);
    return text.substr(first, last - first + 1);
}

std::string normalizeKey(std::string key) {
    for (char& c : key) {
        if (c == '-') {
            c = '_';
        } else {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
    }
    return key;
}

std::string normalizeLaw(std::string law) {
    for (char& c : law) {
        if (c == '_') {
            c = '-';
        } else {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
    }
    return law;
}

bool parseNumber(const std::string& text, double& out) {
    if (text.empty()) {
        return false;
    }
    char* end = nullptr;
    out = std::strtod(text.c_str(), &end);
    return end == text.c_str() + text.size() && std::isfinite(out);
}

}

bool applySetting(Params& p, const std::string& key, const std::string& value, std::string& error) {
    const std::string name = normalizeKey(key);

    if (name == "guidance") {
        const std::string law = normalizeLaw(value);
        if (law == "constant-decel") {
            p.guidance = GuidanceType::ConstantDecel;
        } else if (law == "late-burn") {
            p.guidance = GuidanceType::LateBurn;
        } else {
            error = "guidance must be constant-decel or late-burn, got '" + value + "'";
            return false;
        }
        return true;
    }

    if (name == "output_file") {
        if (value.empty()) {
            error = "output_file must not be empty";
            return false;
        }
        p.output_file = value;
        return true;
    }

    const auto it = numericSettings().find(name);
    if (it == numericSettings().end()) {
        error = "unknown setting '" + key + "'";
        return false;
    }

    double number = 0.0;
    if (!parseNumber(value, number)) {
        error = "'" + value + "' is not a valid number for " + name;
        return false;
    }
    p.*(it->second) = number;
    return true;
}

bool loadConfigFile(const std::string& path, Params& p, std::string& error) {
    std::ifstream file(path);
    if (!file.is_open()) {
        error = "cannot open config file '" + path + "'";
        return false;
    }

    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos) {
            line = line.substr(0, hash);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const std::string where = path + " line " + std::to_string(line_number) + ": ";
        const std::size_t equals = line.find('=');
        if (equals == std::string::npos) {
            error = where + "expected key = value";
            return false;
        }

        std::string problem;
        if (!applySetting(p, trim(line.substr(0, equals)), trim(line.substr(equals + 1)), problem)) {
            error = where + problem;
            return false;
        }
    }
    return true;
}

bool validate(const Params& p, std::string& error) {
    struct Rule {
        bool ok;
        const char* message;
    };
    const Rule rules[] = {
        {p.gravity > 0.0, "gravity must be greater than 0"},
        {p.dry_mass > 0.0, "dry_mass must be greater than 0"},
        {p.fuel_mass >= 0.0, "fuel_mass must not be negative"},
        {p.max_thrust > 0.0, "max_thrust must be greater than 0"},
        {p.min_throttle >= 0.0 && p.min_throttle <= 1.0, "min_throttle must be between 0 and 1"},
        {p.mdot_max >= 0.0, "mdot_max must not be negative"},
        {p.max_tilt_deg >= 0.0 && p.max_tilt_deg < 90.0, "max_tilt_deg must be at least 0 and less than 90"},
        {p.init_altitude > 0.0, "init_altitude must be greater than 0"},
        {p.touchdown_speed >= 0.0, "touchdown_speed must not be negative"},
        {p.const_decel > 0.0, "const_decel must be greater than 0"},
        {p.burn_safety > 0.0 && p.burn_safety <= 1.0, "burn_safety must be above 0 and at most 1"},
        {p.max_vx_cmd >= 0.0, "max_vx_cmd must not be negative"},
        {p.max_land_speed >= 0.0, "max_land_speed must not be negative"},
        {p.max_land_vx >= 0.0, "max_land_vx must not be negative"},
        {p.max_land_dx >= 0.0, "max_land_dx must not be negative"},
        {p.dt > 0.0, "dt must be greater than 0"},
        {p.max_time > 0.0, "max_time must be greater than 0"}
    };
    for (const Rule& rule : rules) {
        if (!rule.ok) {
            error = rule.message;
            return false;
        }
    }
    return true;
}

bool parseArgs(const std::vector<std::string>& args, Params& p, bool& show_help, std::string& error) {
    show_help = false;
    std::vector<std::pair<std::string, std::string>> settings;
    std::string config_path;

    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--help" || arg == "-h") {
            show_help = true;
            continue;
        }
        if (arg.rfind("--", 0) != 0) {
            error = "unexpected argument '" + arg + "' (settings start with --)";
            return false;
        }

        std::string key = arg.substr(2);
        std::string value;
        const std::size_t equals = key.find('=');
        if (equals != std::string::npos) {
            value = key.substr(equals + 1);
            key = key.substr(0, equals);
        } else {
            if (i + 1 >= args.size()) {
                error = "missing value for --" + key;
                return false;
            }
            value = args[++i];
        }

        if (normalizeKey(key) == "config") {
            config_path = value;
        } else {
            settings.emplace_back(key, value);
        }
    }

    if (show_help) {
        return true;
    }
    if (!config_path.empty() && !loadConfigFile(config_path, p, error)) {
        return false;
    }
    for (const auto& setting : settings) {
        if (!applySetting(p, setting.first, setting.second, error)) {
            return false;
        }
    }
    return validate(p, error);
}

std::string usageText() {
    const Params defaults;
    std::ostringstream out;
    out << "Usage: lander [--config FILE] [--setting value ...]\n\n";
    out << "Settings can go in a config file (key = value, one per line, # starts a comment)\n";
    out << "or on the command line (--key value or --key=value). Flags override the file,\n";
    out << "and the file overrides the built-in defaults.\n\n";
    out << "  --guidance        constant-decel | late-burn (default constant-decel)\n";
    out << "  --output_file     CSV path (default " << defaults.output_file << ")\n";
    for (const auto& entry : numericSettings()) {
        out << "  --" << entry.first << "  (default " << defaults.*(entry.second) << ")\n";
    }
    out << "  --help            show this text\n";
    return out.str();
}