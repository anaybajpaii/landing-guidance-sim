#include <iostream>
#include <string>
#include <vector>
#include "config.hpp"
#include "params.hpp"
#include "runner.hpp"

int main(int argc, char** argv) {
    Params p;
    bool show_help = false;
    std::string error;
    std::vector<std::string> runs;
    const std::vector<std::string> args(argv + 1, argv + argc);
    if (!parseArgs(args, p, show_help, runs, error)) {
        std::cerr << "Error: " << error << "\nRun with --help to see the settings.\n";
        return 2;
    }
    if (show_help) {
        std::cout << usageText();
        return 0;
    }

    if (runs.empty()) {
        TouchdownResult result;
        if (!runScenario(p, result, error)) {
            std::cerr << error << "\n";
            return 1;
        }
        printResult(std::cout, p, result);
        return 0;
    }

    std::vector<RunPlan> plan;
    if (!buildRuns(p, runs, plan, error)) {
        std::cerr << "Error: " << error << "\nRun with --help to see the settings.\n";
        return 2;
    }

    std::vector<RunResult> results;
    for (const RunPlan& run : plan) {
        RunResult r;
        r.label = run.label;
        r.params = run.params;
        if (!runScenario(run.params, r.touchdown, error)) {
            std::cerr << error << "\n";
            return 1;
        }
        results.push_back(r);
    }

    if (!writeSummary(p.summary_file, results, error)) {
        std::cerr << error << "\n";
        return 1;
    }

    printTable(std::cout, results);
    std::cout << "\nsummary written to: " << p.summary_file << "\n";
    if (plan.size() == 1) {
        std::cout << "run log:            " << plan.front().params.output_file << "\n";
    } else {
        std::cout << "run logs:           " << plan.front().params.output_file
                  << " ... " << plan.back().params.output_file << "\n";
    }
    return 0;
}