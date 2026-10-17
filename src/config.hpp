#pragma once
#include <string>
#include <vector>
#include "params.hpp"

bool applySetting(Params& p, const std::string& key, const std::string& value, std::string& error);
bool loadConfigFile(const std::string& path, Params& p, std::string& error);
bool validate(const Params& p, std::string& error);
bool parseArgs(const std::vector<std::string>& args, Params& p, bool& show_help, std::string& error);
std::string usageText();