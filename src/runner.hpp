#pragma once
#include <ostream>
#include <string>
#include <vector>
#include "params.hpp"
#include "touchdown.hpp"

struct RunPlan {
    std::string label;
    Params params;
};

struct RunResult {
    std::string label;
    Params params;
    TouchdownResult touchdown;
};

const char* guidanceName(GuidanceType type);
std::string numberedFile(const std::string& path, int index);
bool buildRuns(const Params& base, const std::vector<std::string>& specs,
               std::vector<RunPlan>& plan, std::string& error);
bool runScenario(const Params& p, TouchdownResult& result, std::string& error);
bool writeSummary(const std::string& path, const std::vector<RunResult>& results, std::string& error);
void printResult(std::ostream& out, const Params& p, const TouchdownResult& r);
void printTable(std::ostream& out, const std::vector<RunResult>& results);