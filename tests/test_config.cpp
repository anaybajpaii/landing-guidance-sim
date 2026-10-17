#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "config.hpp"
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

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

const char* kTempFile = "test_config_tmp.cfg";

void writeFile(const std::string& text) {
    std::ofstream file(kTempFile);
    file << text;
}

void testNumbers() {
    Params p;
    std::string error;

    check(applySetting(p, "gravity", "3.71", error) && near(p.gravity, 3.71, 1e-12),
          "number: plain decimal");
    check(applySetting(p, "dt", "1e-3", error) && near(p.dt, 0.001, 1e-12),
          "number: scientific notation");
    check(applySetting(p, "init_vy", "-25", error) && near(p.init_vy, -25.0, 1e-12),
          "number: negative value");
    check(applySetting(p, "init-altitude", "800", error) && near(p.init_altitude, 800.0, 1e-12),
          "number: hyphens work in place of underscores");
    check(applySetting(p, "GRAVITY", "9.81", error) && near(p.gravity, 9.81, 1e-12),
          "number: setting names are not case sensitive");
}

void testBadNumbers() {
    Params p;
    std::string error;

    check(!applySetting(p, "gravity", "abc", error) && contains(error, "not a valid number"),
          "bad number: text is rejected");
    check(!applySetting(p, "gravity", "3.71x", error), "bad number: trailing characters are rejected");
    check(!applySetting(p, "gravity", "", error), "bad number: empty value is rejected");
    check(!applySetting(p, "gravity", "nan", error), "bad number: nan is rejected");
    check(!applySetting(p, "gravity", "inf", error), "bad number: inf is rejected");
    check(near(p.gravity, Params().gravity, 1e-12), "bad number: a rejected value leaves the setting unchanged");
}

void testGuidanceAndOutput() {
    Params p;
    std::string error;

    check(applySetting(p, "guidance", "late-burn", error) && p.guidance == GuidanceType::LateBurn,
          "guidance: late-burn");
    check(applySetting(p, "guidance", "constant-decel", error) && p.guidance == GuidanceType::ConstantDecel,
          "guidance: constant-decel");
    check(applySetting(p, "guidance", "Late_Burn", error) && p.guidance == GuidanceType::LateBurn,
          "guidance: underscores and capitals are accepted");
    check(!applySetting(p, "guidance", "fast", error) && contains(error, "constant-decel or late-burn"),
          "guidance: unknown law is rejected");
    check(applySetting(p, "output_file", "data/other.csv", error) && p.output_file == "data/other.csv",
          "output_file: path is stored as given");
    check(!applySetting(p, "output_file", "", error), "output_file: empty path is rejected");
    check(!applySetting(p, "gravty", "1", error) && contains(error, "unknown setting"),
          "unknown setting is rejected");
}

void testValidate() {
    std::string error;
    check(validate(Params(), error), "validate: the defaults are valid");

    {
        Params p;
        p.dt = 0.0;
        check(!validate(p, error) && contains(error, "dt"), "validate: dt of zero is rejected");
    }
    {
        Params p;
        p.dry_mass = 0.0;
        check(!validate(p, error) && contains(error, "dry_mass"), "validate: dry_mass of zero is rejected");
    }
    {
        Params p;
        p.min_throttle = 1.5;
        check(!validate(p, error) && contains(error, "min_throttle"), "validate: min_throttle above 1 is rejected");
    }
    {
        Params p;
        p.burn_safety = 0.0;
        check(!validate(p, error) && contains(error, "burn_safety"), "validate: burn_safety of zero is rejected");
    }
    {
        Params p;
        p.max_tilt_deg = 90.0;
        check(!validate(p, error) && contains(error, "max_tilt_deg"), "validate: a 90 degree tilt limit is rejected");
    }
    {
        Params p;
        p.fuel_mass = -1.0;
        check(!validate(p, error) && contains(error, "fuel_mass"), "validate: negative fuel is rejected");
    }
}

void testConfigFile() {
    writeFile(
        "# a comment line\n"
        "\n"
        "gravity = 3.71\n"
        "  max_thrust=9000   # trailing comment\n"
        "guidance = late-burn\n"
        "output_file = data/mars.csv\n");

    Params p;
    std::string error;
    check(loadConfigFile(kTempFile, p, error), "file: a valid file loads");
    check(near(p.gravity, 3.71, 1e-12) && near(p.max_thrust, 9000.0, 1e-12),
          "file: numbers are read, with or without spaces around =");
    check(p.guidance == GuidanceType::LateBurn && p.output_file == "data/mars.csv",
          "file: guidance and output file are read");
    check(near(p.dry_mass, Params().dry_mass, 1e-12), "file: settings not in the file keep their defaults");
}

void testConfigFileErrors() {
    Params p;
    std::string error;

    check(!loadConfigFile("no_such_file.cfg", p, error) && contains(error, "cannot open"),
          "file error: a missing file is reported");

    writeFile("gravity = 3.71\nthis line has no equals sign\n");
    check(!loadConfigFile(kTempFile, p, error) && contains(error, "line 2"),
          "file error: a line without = names its line number");

    writeFile("gravity = 3.71\n\ngravty = 4\n");
    check(!loadConfigFile(kTempFile, p, error) && contains(error, "line 3") && contains(error, "unknown setting"),
          "file error: an unknown key names its line number");
}

void testArgs() {
    Params p;
    bool help = false;
    std::string error;

    const std::vector<std::string> args = {"--gravity", "3.71", "--guidance=late-burn", "--init_vy", "-25"};
    check(parseArgs(args, p, help, error), "args: valid flags are accepted");
    check(near(p.gravity, 3.71, 1e-12) && p.guidance == GuidanceType::LateBurn && near(p.init_vy, -25.0, 1e-12),
          "args: both --key value and --key=value forms work, including negatives");
    check(!help, "args: help is off unless asked for");
}

void testPrecedence() {
    writeFile("gravity = 3.71\ndt = 0.02\n");

    Params p;
    bool help = false;
    std::string error;
    const std::vector<std::string> args = {"--gravity", "9.81", "--config", kTempFile};
    check(parseArgs(args, p, help, error), "precedence: flags and a config file can be combined");
    check(near(p.gravity, 9.81, 1e-12), "precedence: a flag beats the file, even when it comes first");
    check(near(p.dt, 0.02, 1e-12), "precedence: the file beats the built-in default");
}

void testArgErrors() {
    bool help = false;
    std::string error;

    {
        Params p;
        check(!parseArgs({"--gravity"}, p, help, error) && contains(error, "missing value"),
              "arg error: a flag without a value is rejected");
    }
    {
        Params p;
        check(!parseArgs({"gravity"}, p, help, error) && contains(error, "unexpected argument"),
              "arg error: a bare word is rejected");
    }
    {
        Params p;
        check(!parseArgs({"--gravty", "1"}, p, help, error) && contains(error, "unknown setting"),
              "arg error: an unknown flag is rejected");
    }
    {
        Params p;
        check(!parseArgs({"--dt", "0"}, p, help, error) && contains(error, "dt"),
              "arg error: values are validated after parsing");
    }
    {
        Params p;
        check(!parseArgs({"--config", "no_such_file.cfg"}, p, help, error) && contains(error, "cannot open"),
              "arg error: a missing config file is reported");
    }
}

void testHelp() {
    Params p;
    bool help = false;
    std::string error;
    check(parseArgs({"--help"}, p, help, error) && help, "help: --help is recognised");
    check(parseArgs({"-h"}, p, help, error) && help, "help: -h is recognised");

    const std::string text = usageText();
    check(contains(text, "--gravity") && contains(text, "--guidance") && contains(text, "--max_time"),
          "help: the usage text lists the settings");
}

}

int main() {
    testNumbers();
    testBadNumbers();
    testGuidanceAndOutput();
    testValidate();
    testConfigFile();
    testConfigFileErrors();
    testArgs();
    testPrecedence();
    testArgErrors();
    testHelp();

    std::remove(kTempFile);

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}