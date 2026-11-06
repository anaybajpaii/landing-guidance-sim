#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "params.hpp"
#include "runner.hpp"

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

const char* kRunFile = "test_runner_tmp.csv";
const char* kSummaryFile = "test_runner_summary_tmp.csv";

std::vector<std::string> readLines(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }
    return lines;
}

Params testParams() {
    Params p;
    p.output_file = kRunFile;
    return p;
}

void testNumberedFile() {
    check(numberedFile("data/run.csv", 2) == "data/run_2.csv", "numbered: folder and extension");
    check(numberedFile("run.csv", 1) == "run_1.csv", "numbered: no folder");
    check(numberedFile("out", 3) == "out_3", "numbered: no extension");
    check(numberedFile("my.dir/out", 3) == "my.dir/out_3", "numbered: a dot in the folder name is not an extension");
    check(numberedFile("a/b.c/run.csv", 12) == "a/b.c/run_12.csv", "numbered: two digit index");
}

void testBuildRuns() {
    Params base;
    std::vector<RunPlan> plan;
    std::string error;

    check(buildRuns(base, {"guidance=late-burn", "kp_vert=4 ki_vert=0.4"}, plan, error),
          "build: valid specs are accepted");
    check(plan.size() == 2, "build: one plan per spec");
    check(plan[0].label == "guidance=late-burn" && plan[1].label == "kp_vert=4 ki_vert=0.4",
          "build: the label is the spec text");
    check(plan[0].params.guidance == GuidanceType::LateBurn, "build: the first run changes the guidance law");
    check(plan[1].params.guidance == GuidanceType::ConstantDecel
              && near(plan[1].params.kp_vert, 4.0, 1e-12) && near(plan[1].params.ki_vert, 0.4, 1e-12),
          "build: the second run changes only its own settings");
    check(plan[0].params.output_file == "data/run_1.csv" && plan[1].params.output_file == "data/run_2.csv",
          "build: each run gets its own numbered CSV file");
    check(base.guidance == GuidanceType::ConstantDecel && near(base.kp_vert, 2.0, 1e-12),
          "build: the base settings are not modified");
}

void testBuildRunsBase() {
    Params base;
    base.gravity = 3.71;
    std::vector<RunPlan> plan;
    std::string error;

    check(buildRuns(base, {"", "   "}, plan, error) && plan.size() == 2, "build: blank specs are allowed");
    check(plan[0].label == "base" && near(plan[0].params.gravity, 3.71, 1e-12),
          "build: a blank spec is the base settings, labelled base");
}

void testBuildRunsErrors() {
    Params base;
    std::vector<RunPlan> plan;
    std::string error;

    check(!buildRuns(base, {"kp_vert=4", "gravty=1"}, plan, error)
              && contains(error, "run 2") && contains(error, "unknown setting"),
          "build error: an unknown setting names the run number");
    check(!buildRuns(base, {"dt=0"}, plan, error) && contains(error, "run 1") && contains(error, "dt"),
          "build error: an invalid value names the run number");
    check(!buildRuns(base, {"kp_vert"}, plan, error) && contains(error, "expected key=value"),
          "build error: a word without = is rejected");
    check(!buildRuns(base, {"output_file=other.csv"}, plan, error) && contains(error, "output_file"),
          "build error: runs cannot choose their own output file");
}

void testRunScenario() {
    Params p = testParams();
    TouchdownResult result;
    std::string error;

    check(runScenario(p, result, error), "run: the default scenario runs");
    check(result.landed && result.success, "run: the default scenario lands successfully");
    check(near(result.time, 31.09, 0.05), "run: landing time matches the earlier commits");
    check(near(result.vertical_speed, 0.85, 0.02), "run: landing speed matches the earlier commits");

    const std::vector<std::string> lines = readLines(kRunFile);
    check(lines.size() > 1000 && lines[0] == "t,x,y,vx,vy,fuel,mass,thrust,tilt_deg",
          "run: the CSV has a header and one row per step");

    TouchdownResult again;
    check(runScenario(p, again, error) && near(again.time, result.time, 1e-12)
              && near(again.fuel_used, result.fuel_used, 1e-12),
          "run: the same settings give the same result every time");
}

void testRunScenarioVariants() {
    Params gentle = testParams();
    Params late = testParams();
    late.guidance = GuidanceType::LateBurn;
    TouchdownResult a;
    TouchdownResult b;
    std::string error;

    check(runScenario(gentle, a, error) && runScenario(late, b, error), "variants: both laws run");
    check(b.success && b.fuel_used < a.fuel_used && b.time < a.time,
          "variants: late burn lands sooner and uses less fuel");

    Params far = testParams();
    far.guidance = GuidanceType::LateBurn;
    far.init_x = 100.0;
    TouchdownResult c;
    check(runScenario(far, c, error) && !c.success && !c.position_ok,
          "variants: late burn from 100 m away misses the pad");

    Params slow = testParams();
    slow.max_time = 5.0;
    TouchdownResult d;
    check(runScenario(slow, d, error) && !d.landed && !d.success, "variants: running out of time is not a landing");
}

void testRunScenarioError() {
    Params p;
    p.output_file = "no_such_folder_for_tests/run.csv";
    TouchdownResult result;
    std::string error;
    check(!runScenario(p, result, error) && contains(error, "Could not open"),
          "run error: an unwritable CSV path is reported");
}

void testSummary() {
    Params p = testParams();
    RunResult one;
    one.label = "guidance=constant-decel";
    one.params = p;
    std::string error;
    runScenario(one.params, one.touchdown, error);

    RunResult two;
    two.label = "a,b";
    two.params = p;
    two.params.guidance = GuidanceType::LateBurn;
    runScenario(two.params, two.touchdown, error);

    check(writeSummary(kSummaryFile, {one, two}, error), "summary: the file is written");
    const std::vector<std::string> lines = readLines(kSummaryFile);
    check(lines.size() == 3, "summary: a header plus one row per run");
    check(lines[0].rfind("run,label,guidance,", 0) == 0, "summary: the header names the columns");
    check(contains(lines[1], ",constant-decel,2,0.2,0.1,1,1,"), "summary: a row holds guidance, gains, landed and success");
    check(lines[1].rfind("1,guidance=constant-decel,", 0) == 0, "summary: rows are numbered from 1");
    check(contains(lines[2], "\"a,b\""), "summary: a label containing a comma is quoted");
    check(contains(lines[2], ",late-burn,"), "summary: the guidance law is recorded per run");

    check(!writeSummary("no_such_folder_for_tests/summary.csv", {one}, error) && contains(error, "Could not open"),
          "summary: an unwritable path is reported");
}

void testPrintTable() {
    Params p = testParams();
    RunResult good;
    good.label = "good";
    good.params = p;
    std::string error;
    runScenario(good.params, good.touchdown, error);

    RunResult bad;
    bad.label = "bad";
    bad.params = p;
    bad.params.guidance = GuidanceType::LateBurn;
    bad.params.init_x = 100.0;
    runScenario(bad.params, bad.touchdown, error);

    RunResult timeout;
    timeout.label = "timeout";
    timeout.params = p;
    timeout.params.max_time = 5.0;
    runScenario(timeout.params, timeout.touchdown, error);

    std::ostringstream out;
    printTable(out, {good, bad, timeout});
    const std::string text = out.str();
    check(contains(text, "SUCCESS") && contains(text, "FAIL") && contains(text, "TIMEOUT"),
          "table: each run shows its verdict");
    check(contains(text, "good") && contains(text, "bad") && contains(text, "timeout"),
          "table: each run shows its label");
}

void testPrintResult() {
    Params p = testParams();
    TouchdownResult result;
    std::string error;
    runScenario(p, result, error);

    std::ostringstream out;
    printResult(out, p, result);
    const std::string text = out.str();
    check(contains(text, "Result: SUCCESS") && contains(text, "landing speed:"),
          "print: a single run shows its verdict and landing speed");
    check(contains(text, "guidance: constant-decel, gravity 1.62 m/s^2"),
          "print: the guidance law and gravity are shown");
}

}

int main() {
    testNumberedFile();
    testBuildRuns();
    testBuildRunsBase();
    testBuildRunsErrors();
    testRunScenario();
    testRunScenarioVariants();
    testRunScenarioError();
    testSummary();
    testPrintTable();
    testPrintResult();

    std::remove(kRunFile);
    std::remove(kSummaryFile);

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}